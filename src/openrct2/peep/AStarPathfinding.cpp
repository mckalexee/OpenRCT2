/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "AStarPathfinding.h"

#include "../Diagnostic.h"
#include "../entity/Peep.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/tile_element/BannerElement.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TrackElement.h"

#include <cstdlib>
#include <queue>
#include <unordered_map>

namespace OpenRCT2::PathFinding
{
#pragma region A* Pathfinding Logging
    // Set to true to enable A* pathfinding logging.
    static constexpr bool kLogAStarPathfinding = true;

    // Only log for guests with this name (set empty to log all)
    static constexpr const char* kLogAStarPeepName = "Pete Pathfinder";

    template<typename... TArgs>
    static void LogAStar(
        [[maybe_unused]] const Peep* peep, [[maybe_unused]] const char* format, [[maybe_unused]] TArgs&&... args)
    {
        if constexpr (kLogAStarPathfinding)
        {
            if (peep != nullptr && kLogAStarPeepName[0] != '\0' && peep->GetName() != kLogAStarPeepName)
                return;

            char buffer[256];
            snprintf(buffer, sizeof(buffer), format, std::forward<TArgs>(args)...);

            if (peep != nullptr)
            {
                LOG_INFO("[A*][%05u:%s] %s", peep->Id.ToUnderlying(), peep->GetName().c_str(), buffer);
            }
            else
            {
                LOG_INFO("[A*] %s", buffer);
            }
        }
    }
#pragma endregion

    // Maximum number of tiles to explore before giving up
    static constexpr int32_t kAStarMaxTiles = 500000;

    struct AStarNode
    {
        TileCoordsXYZ position;
        int32_t gCost;           // Cost from start
        int32_t hCost;           // Heuristic to goal
        Direction firstDirection; // First step direction from start (the return value)

        int32_t fCost() const
        {
            return gCost + hCost;
        }
    };

    // Comparison for priority queue - lower f-cost has higher priority, tiebreak by lower g-cost
    struct AStarNodeCompare
    {
        bool operator()(const AStarNode& a, const AStarNode& b) const
        {
            int32_t fA = a.fCost();
            int32_t fB = b.fCost();
            if (fA != fB)
                return fA > fB; // Min-heap: lower f-cost = higher priority
            return a.gCost > b.gCost; // Prefer nodes with lower g-cost
        }
    };

    // Pack position into uint64_t for use as map key
    struct PositionKey
    {
        uint64_t value;

        explicit PositionKey(const TileCoordsXYZ& pos)
            : value(static_cast<uint64_t>(pos.x) | (static_cast<uint64_t>(pos.y) << 16)
                    | (static_cast<uint64_t>(pos.z) << 32))
        {
        }

        bool operator==(const PositionKey& other) const
        {
            return value == other.value;
        }
    };

    struct PositionKeyHash
    {
        std::size_t operator()(const PositionKey& key) const
        {
            return std::hash<uint64_t>{}(key.value);
        }
    };

    // Heuristic weight > 1.0 trades optimality for speed (weighted A*)
    // This helps complex mazes by preferring paths that head toward the goal
    static constexpr float kHeuristicWeight = 1.5f;

    // Direction orders rotated by guest ID for path variance
    // Each guest consistently prefers a different direction order, creating visual spread
    static constexpr Direction kDirectionOrders[4][4] = {
        { 0, 1, 2, 3 },
        { 1, 2, 3, 0 },
        { 2, 3, 0, 1 },
        { 3, 0, 1, 2 },
    };

    // Manhattan distance heuristic with weight
    static int32_t CalculateHeuristic(const TileCoordsXYZ& from, const TileCoordsXYZ& to)
    {
        int32_t manhattan = std::abs(from.x - to.x) + std::abs(from.y - to.y);
        return static_cast<int32_t>(manhattan * kHeuristicWeight);
    }

    // Check if a path element is valid for A* pathfinding from the given direction
    static const PathElement* GetValidPathElement(
        const TileCoordsXYZ& pos, Direction fromDirection, bool ignoreForeignQueues, RideId queueRideIndex)
    {
        TileElement* tileElement = MapGetFirstElementAt(pos);
        if (tileElement == nullptr)
            return nullptr;

        do
        {
            if (tileElement->IsGhost())
                continue;

            if (tileElement->GetType() != TileElementType::Path)
                continue;

            const auto* pathElement = tileElement->AsPath();

            // Check if the path is at the correct height for the given direction
            if (!FootpathIsZAndDirectionValid(*pathElement, pos.z, fromDirection))
                continue;

            // Check for foreign queue restriction
            if (ignoreForeignQueues && pathElement->IsQueue())
            {
                RideId pathRideIndex = pathElement->GetRideIndex();
                if (!pathRideIndex.IsNull() && pathRideIndex != queueRideIndex)
                {
                    // This is a queue for a different ride - treat as unwalkable
                    continue;
                }
            }

            return pathElement;
        } while (!(tileElement++)->IsLastForTile());

        return nullptr;
    }

    // Get the arrival Z coordinate when moving from current tile in the given direction
    static int32_t GetArrivalZ(const PathElement* currentPath, Direction direction)
    {
        int32_t arrivalZ = currentPath->BaseHeight;
        if (currentPath->IsSloped() && currentPath->GetSlopeDirection() == direction)
        {
            arrivalZ += 2;
        }
        return arrivalZ;
    }

    // Get the banner element on top of a path element (for no-entry signs)
    static const TileElement* GetBannerOnPath(const TileElement* pathElement)
    {
        if (pathElement->IsLastForTile())
            return nullptr;

        const TileElement* bannerElement = pathElement + 1;
        do
        {
            // Path on top, so no banners
            if (bannerElement->GetType() == TileElementType::Path)
                return nullptr;
            // Found a banner
            if (bannerElement->GetType() == TileElementType::Banner)
                return bannerElement;
            // Last element so there can't be any other banners
            if (bannerElement->IsLastForTile())
                return nullptr;

        } while (bannerElement++ != nullptr);

        return nullptr;
    }

    // Remove edges blocked by no-entry banners
    static int32_t BannerClearPathEdges(const PathElement* pathElement, int32_t edges)
    {
        const TileElement* bannerElement = GetBannerOnPath(reinterpret_cast<const TileElement*>(pathElement));
        if (bannerElement != nullptr)
        {
            do
            {
                edges &= bannerElement->AsBanner()->GetAllowedEdges();
            } while ((bannerElement = GetBannerOnPath(bannerElement)) != nullptr);
        }
        return edges;
    }

    // Get the permitted edges of a path element (edges without no-entry signs)
    static uint8_t GetPermittedEdges(const PathElement* pathElement)
    {
        return BannerClearPathEdges(pathElement, pathElement->GetEdgesAndCorners()) & 0x0F;
    }

    // Check if an entrance can be approached from a given direction
    static bool IsEntranceApproachableFromDirection(const EntranceElement* entrance, Direction approachDirection)
    {
        // The entrance faces a certain direction; we can approach from that direction
        Direction entranceDirection = entrance->GetDirection();
        return approachDirection == entranceDirection;
    }

    // Check if a tile at the goal position is a valid destination (shop, park exit, ride entrance)
    // This handles non-path goal tiles that A* wouldn't otherwise recognize
    static bool IsGoalTileReachable(const TileCoordsXYZ& pos, const TileCoordsXYZ& goal, Direction approachDirection)
    {
        if (pos.x != goal.x || pos.y != goal.y)
            return false;

        TileElement* tileElement = MapGetFirstElementAt(goal);
        if (tileElement == nullptr)
            return false;

        do
        {
            if (tileElement->IsGhost())
                continue;

            // Check for shop/facility entrance (Track element)
            if (tileElement->GetType() == TileElementType::Track)
            {
                if (tileElement->BaseHeight != goal.z)
                    continue;

                auto* trackElement = tileElement->AsTrack();
                auto rideIndex = trackElement->GetRideIndex();
                auto* ride = GetRide(rideIndex);
                if (ride != nullptr && ride->getRideTypeDescriptor().HasFlag(RtdFlag::isShopOrFacility))
                {
                    return true;
                }
            }
            // Check for park entrance/exit or ride entrance/exit (Entrance element)
            else if (tileElement->GetType() == TileElementType::Entrance)
            {
                if (tileElement->BaseHeight != goal.z)
                    continue;

                auto* entranceElement = tileElement->AsEntrance();
                auto entranceType = entranceElement->GetEntranceType();

                if (entranceType == ENTRANCE_TYPE_PARK_ENTRANCE)
                {
                    return true;
                }
                else if (entranceType == ENTRANCE_TYPE_RIDE_ENTRANCE || entranceType == ENTRANCE_TYPE_RIDE_EXIT)
                {
                    // Check if we're approaching from the correct direction
                    if (IsEntranceApproachableFromDirection(entranceElement, approachDirection))
                    {
                        return true;
                    }
                }
            }
        } while (!(tileElement++)->IsLastForTile());

        return false;
    }

    Direction AStarChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, const Peep& peep, bool ignoreForeignQueues,
        RideId queueRideIndex)
    {
        // Already at goal
        if (loc == goal)
            return kInvalidDirection;

        // Find the path element at the start position
        const PathElement* startPath = nullptr;
        TileElement* startElement = MapGetFirstElementAt(loc);
        if (startElement == nullptr)
            return kInvalidDirection;

        do
        {
            if (startElement->IsGhost())
                continue;
            if (startElement->GetType() != TileElementType::Path)
                continue;
            if (startElement->BaseHeight != loc.z)
                continue;

            startPath = startElement->AsPath();
            break;
        } while (!(startElement++)->IsLastForTile());

        if (startPath == nullptr)
            return kInvalidDirection;

        // Get permitted edges from start
        uint8_t startEdges = GetPermittedEdges(startPath);
        if (startEdges == 0)
            return kInvalidDirection;

        // Exclude the direction the peep came from to prevent oscillation on wide paths
        // (peep.PeepDirection is the direction the peep is currently facing/moving)
        if (DirectionValid(peep.PeepDirection))
        {
            Direction reverseDir = DirectionReverse(peep.PeepDirection);
            uint8_t edgesWithoutReverse = startEdges & ~(1 << reverseDir);

            // Only exclude reverse if other edges are available (allow backtracking if stuck)
            if (edgesWithoutReverse != 0)
                startEdges = edgesWithoutReverse;
        }

        // Initialize open set (priority queue) and closed set
        std::priority_queue<AStarNode, std::vector<AStarNode>, AStarNodeCompare> openSet;
        std::unordered_map<PositionKey, int32_t, PositionKeyHash> closedSet;

        // Add starting position to closed set to prevent backtracking through it.
        // Without this, A* could explore a dead end, backtrack to start with
        // firstDirection pointing toward the dead end, then find the goal via
        // a different path but return the wrong firstDirection.
        closedSet[PositionKey(loc)] = 0;

        LogAStar(&peep, "Start (%d,%d,%d) -> Goal (%d,%d,%d)", loc.x, loc.y, loc.z, goal.x, goal.y, goal.z);

        int32_t tilesExplored = 0;

        // Select direction order based on guest ID for consistent path variance
        const auto& dirOrder = kDirectionOrders[peep.Id.ToUnderlying() % 4];

        // Add initial neighbors to open set
        for (Direction dir : dirOrder)
        {
            if (!(startEdges & (1 << dir)))
                continue;

            // Calculate arrival position
            TileCoordsXYZ neighborPos;
            neighborPos.x = loc.x + TileDirectionDelta[dir].x;
            neighborPos.y = loc.y + TileDirectionDelta[dir].y;
            neighborPos.z = GetArrivalZ(startPath, dir);

            // Check if this is the goal tile (shop, park exit, ride entrance)
            if (IsGoalTileReachable(neighborPos, goal, dir))
            {
                LogAStar(&peep, "SUCCESS (initial): dir=%d, goal is adjacent", dir);
                return dir;
            }

            // Check if neighbor is valid and has return edge
            const PathElement* neighborPath = GetValidPathElement(neighborPos, dir, ignoreForeignQueues, queueRideIndex);
            if (neighborPath == nullptr)
                continue;

            // Check for return edge
            Direction reverseDir = DirectionReverse(dir);
            if (!(neighborPath->GetEdges() & (1 << reverseDir)))
                continue;

            // Adjust Z to path base height after validation
            neighborPos.z = neighborPath->BaseHeight;

            AStarNode node;
            node.position = neighborPos;
            node.gCost = 1;
            node.hCost = CalculateHeuristic(neighborPos, goal);
            node.firstDirection = dir;

            openSet.push(node);
        }

        // A* main loop
        while (!openSet.empty() && tilesExplored < kAStarMaxTiles)
        {
            AStarNode current = openSet.top();
            openSet.pop();

            // Check if we reached the goal
            if (current.position == goal)
            {
                LogAStar(&peep, "SUCCESS: dir=%d, gCost=%d, tiles explored=%d", current.firstDirection, current.gCost, tilesExplored);
                return current.firstDirection;
            }

            // Check if already visited with a better cost
            PositionKey posKey(current.position);
            auto it = closedSet.find(posKey);
            if (it != closedSet.end() && it->second <= current.gCost)
            {
                continue;
            }
            closedSet[posKey] = current.gCost;

            tilesExplored++;
            if constexpr (kLogAStarPathfinding)
            {
                if (tilesExplored % 5000 == 0)
                    LogAStar(&peep, "Progress: %d tiles explored, open set: %zu", tilesExplored, openSet.size());
            }

            // Find path element at current position
            const PathElement* currentPath = nullptr;
            TileElement* currentElement = MapGetFirstElementAt(current.position);
            if (currentElement == nullptr)
                continue;

            do
            {
                if (currentElement->IsGhost())
                    continue;
                if (currentElement->GetType() != TileElementType::Path)
                    continue;
                if (currentElement->BaseHeight != current.position.z)
                    continue;

                currentPath = currentElement->AsPath();
                break;
            } while (!(currentElement++)->IsLastForTile());

            if (currentPath == nullptr)
                continue;

            // Get permitted edges
            uint8_t edges = GetPermittedEdges(currentPath);

            // Explore neighbors
            for (Direction dir : dirOrder)
            {
                if (!(edges & (1 << dir)))
                    continue;

                // Calculate arrival position
                TileCoordsXYZ neighborPos;
                neighborPos.x = current.position.x + TileDirectionDelta[dir].x;
                neighborPos.y = current.position.y + TileDirectionDelta[dir].y;
                neighborPos.z = GetArrivalZ(currentPath, dir);

                // Check if this is the goal tile (shop, park exit, ride entrance)
                if (IsGoalTileReachable(neighborPos, goal, dir))
                {
                    LogAStar(&peep, "SUCCESS: dir=%d, gCost=%d, tiles explored=%d", current.firstDirection, current.gCost + 1, tilesExplored);
                    return current.firstDirection;
                }

                // Check if neighbor is valid
                const PathElement* neighborPath = GetValidPathElement(neighborPos, dir, ignoreForeignQueues, queueRideIndex);
                if (neighborPath == nullptr)
                    continue;

                // Check for return edge
                Direction reverseDir = DirectionReverse(dir);
                if (!(neighborPath->GetEdges() & (1 << reverseDir)))
                    continue;

                // Adjust Z to path base height after validation
                neighborPos.z = neighborPath->BaseHeight;

                // Check if already visited with a better cost
                PositionKey neighborKey(neighborPos);
                int32_t newGCost = current.gCost + 1;
                auto neighborIt = closedSet.find(neighborKey);
                if (neighborIt != closedSet.end() && neighborIt->second <= newGCost)
                {
                    continue;
                }

                AStarNode neighbor;
                neighbor.position = neighborPos;
                neighbor.gCost = newGCost;
                neighbor.hCost = CalculateHeuristic(neighborPos, goal);
                neighbor.firstDirection = current.firstDirection; // Preserve first direction

                openSet.push(neighbor);
            }
        }

        // Goal not found - return invalid direction to fall back to DFS
        LogAStar(&peep, "FAILED: tiles explored=%d, open set empty=%s", tilesExplored, openSet.empty() ? "yes" : "no");
        return kInvalidDirection;
    }

} // namespace OpenRCT2::PathFinding
