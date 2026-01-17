/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "AStarPathfinding.h"

#include "../entity/Peep.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TileElement.h"

#include <cstdlib>
#include <queue>
#include <unordered_map>

namespace OpenRCT2::PathFinding
{
    // Maximum number of tiles to explore before giving up
    static constexpr int32_t kAStarMaxTiles = 5000;

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

    // Manhattan distance heuristic
    static int32_t CalculateHeuristic(const TileCoordsXYZ& from, const TileCoordsXYZ& to)
    {
        return std::abs(from.x - to.x) + std::abs(from.y - to.y);
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

    // Get the permitted edges of a path element (edges without no-entry signs)
    static uint8_t GetPermittedEdges(const PathElement* pathElement)
    {
        // For simplicity, we just get the raw edges
        // In the full implementation, we'd also check for banners
        return pathElement->GetEdges() & 0x0F;
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

        // Initialize open set (priority queue) and closed set
        std::priority_queue<AStarNode, std::vector<AStarNode>, AStarNodeCompare> openSet;
        std::unordered_map<PositionKey, int32_t, PositionKeyHash> closedSet;

        // Add starting position to closed set to prevent backtracking through it.
        // Without this, A* could explore a dead end, backtrack to start with
        // firstDirection pointing toward the dead end, then find the goal via
        // a different path but return the wrong firstDirection.
        closedSet[PositionKey(loc)] = 0;

        int32_t tilesExplored = 0;

        // Add initial neighbors to open set
        for (Direction dir : kAllDirections)
        {
            if (!(startEdges & (1 << dir)))
                continue;

            // Calculate arrival position
            TileCoordsXYZ neighborPos;
            neighborPos.x = loc.x + TileDirectionDelta[dir].x;
            neighborPos.y = loc.y + TileDirectionDelta[dir].y;
            neighborPos.z = GetArrivalZ(startPath, dir);

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
            for (Direction dir : kAllDirections)
            {
                if (!(edges & (1 << dir)))
                    continue;

                // Calculate arrival position
                TileCoordsXYZ neighborPos;
                neighborPos.x = current.position.x + TileDirectionDelta[dir].x;
                neighborPos.y = current.position.y + TileDirectionDelta[dir].y;
                neighborPos.z = GetArrivalZ(currentPath, dir);

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
        return kInvalidDirection;
    }

} // namespace OpenRCT2::PathFinding
