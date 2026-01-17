# A* Pathfinding Implementation Requirements for OpenRCT2

## Overview

OpenRCT2 is an open-source re-implementation of RollerCoaster Tycoon 2. Guests in the park need to navigate footpath networks to reach destinations (ride entrances, exits, shops, park exit). The existing pathfinding uses a depth-first search (DFS) algorithm that has worked reliably for 25+ years. The goal is to implement A* as an optimization for guests who have maps or are leaving the park.

## Game World Basics

### Coordinate System

- The world is a grid of **tiles** (32x32 world units each)
- `TileCoordsXYZ` represents tile coordinates: x, y, z
- Z represents vertical height in "clearance units" (8 world units per Z level)
- Paths are typically at Z=14 for ground level parks

### Footpaths

Footpaths are tile elements of type `TileElementType::Path`. Each path tile has:

- **BaseHeight**: The canonical Z coordinate of the path (low end for slopes)
- **Edges**: A 4-bit mask indicating which directions connect to adjacent paths
  - Bit 0 = North, Bit 1 = East, Bit 2 = South, Bit 3 = West
- **IsSloped()**: Whether this is a ramp
- **GetSlopeDirection()**: For sloped paths, the direction that goes uphill
- **IsQueue()**: Whether this is a queue line for a ride
- **GetRideIndex()**: For queue paths, which ride this queue belongs to

### Direction Constants

```cpp
// Direction values
// 0 = North (+Y direction)
// 1 = East (+X direction)
// 2 = South (-Y direction)
// 3 = West (-X direction)

// Movement deltas
TileDirectionDelta[0] = {0, +1}   // North
TileDirectionDelta[1] = {+1, 0}   // East
TileDirectionDelta[2] = {0, -1}   // South
TileDirectionDelta[3] = {-1, 0}   // West
```

## Sloped Paths (Critical)

Sloped paths are ramps connecting two different Z levels:

- **BaseHeight**: The Z coordinate at the LOW end of the slope
- **BaseHeight + 2**: The Z coordinate at the HIGH end of the slope
- **SlopeDirection**: The direction you would walk to go UPHILL

### Example

A slope with `BaseHeight=10` and `SlopeDirection=2` (South):
- Walking South (direction 2): Enter at Z=10, exit at Z=12 (going uphill)
- Walking North (direction 0): Enter at Z=12, exit at Z=10 (going downhill)

### Validation Function

The function `FootpathIsZAndDirectionValid` in `src/openrct2/world/Footpath.cpp` validates whether a path can be entered at a given Z and direction:

```cpp
bool FootpathIsZAndDirectionValid(const PathElement& pathElement, int32_t currentZ, int32_t currentDirection)
{
    if (pathElement.IsSloped())
    {
        int32_t slopeDirection = pathElement.GetSlopeDirection();
        if (slopeDirection == currentDirection)
        {
            // Walking uphill - must enter at BaseHeight
            if (currentZ != pathElement.BaseHeight)
                return false;
        }
        else
        {
            // Walking downhill - must enter at BaseHeight + 2
            slopeDirection = DirectionReverse(slopeDirection);
            if (slopeDirection != currentDirection)
                return false;
            if (currentZ != pathElement.BaseHeight + 2)
                return false;
        }
    }
    else
    {
        // Flat path - Z must match BaseHeight
        if (currentZ != pathElement.BaseHeight)
            return false;
    }
    return true;
}
```

## The Working DFS Implementation

The existing DFS pathfinding in `GuestPathfinding.cpp` works correctly. Key functions:

### `GuestPathfinding::ChooseDirection`

Entry point that selects the best direction for a guest to walk. For guests with maps or leaving the park, it can use optimized pathfinding (where A* would be used).

### `PeepPathfindSetUp`

Sets up the pathfinding goal and initializes search state.

### `PeepPathfindIsGoal`

Checks if a location matches the pathfinding goal (ride entrance, park exit, etc.).

### `FootpathElementDestInDir`

Core DFS traversal function. Given a current location and direction, finds where you end up. This correctly handles:
- Slope Z transitions
- Edge connectivity
- Queue restrictions

## A* Implementation Requirements

### Graph Structure

The path network forms an implicit graph where:
- **Nodes**: Path tiles identified by (x, y, z) coordinates
- **Edges**: Connections between adjacent paths based on the edge bitmask
- **Cost**: Typically uniform (1 per tile), but could weight slopes or queues differently

### Algorithm Requirements

1. **Open Set**: Priority queue of nodes to explore, ordered by f-cost (g + h)
2. **Closed Set**: Set of already-explored nodes to avoid revisiting
3. **g-cost**: Actual distance from start to current node
4. **h-cost**: Heuristic estimate from current node to goal (Manhattan distance works well)
5. **f-cost**: g + h, used for priority ordering

### Key Implementation Points

1. **Goal Detection**: The goal is typically the tile in front of a ride entrance, not the entrance tile itself. Use `PeepPathfindIsGoal` or check against `Guest::GuestHeadingToRideId`.

2. **Edge Traversal**: When moving from tile A to tile B:
   - Check that A has an edge in the desired direction
   - Check that B exists and has a path element
   - Validate Z using `FootpathIsZAndDirectionValid`
   - Check that B has an edge back towards A (paths are bidirectional)

3. **Z Coordinate Handling**:
   - When expanding neighbors, compute the arrival Z based on current tile's slope
   - Store the canonical BaseHeight in nodes for consistency
   - Use arrival Z only for validation

4. **Queue Handling**: Queue paths may have restrictions:
   - Guests heading to a ride can use that ride's queue
   - "Foreign" queues (belonging to other rides) should typically be avoided

5. **Return Value**: Return the FIRST direction to walk from the starting tile, not the direction at the goal.

### Heuristic Function

Manhattan distance works well for grid-based pathfinding:

```cpp
int heuristic(TileCoordsXYZ current, TileCoordsXYZ goal) {
    return abs(current.x - goal.x) + abs(current.y - goal.y);
}
```

Z difference can optionally be included but may not significantly improve results.

### Failure Handling

When A* cannot find a path to the goal:
- **DO NOT** return a "best guess" direction toward the goal
- Return `kInvalidDirection` to signal failure
- Let the caller fall back to random wandering or DFS

Returning a "closest to goal" direction when no path exists causes guests to walk toward unreachable goals, getting stuck at dead ends.

## Important Gotchas

1. **Paths Can Stack**: Multiple path elements can exist at the same (x, y) with different Z values (bridges, underground paths). Always check Z.

2. **Ghost Elements**: Ignore elements where `IsGhost()` returns true (these are preview elements during construction).

3. **Direction Consistency**: The direction you ENTER a tile is the direction you were WALKING, not where you came FROM. Direction 2 (South) means walking southward.

4. **Dead Ends**: A path tile with only one edge is a dead end. Guests should turn around, not try to walk through walls.

5. **Park Boundaries**: Tiles outside the park boundary may use different pathfinding logic.

## File Locations

- **Main pathfinding code**: `src/openrct2/peep/GuestPathfinding.cpp`
- **Pathfinding header**: `src/openrct2/peep/GuestPathfinding.h`
- **Footpath utilities**: `src/openrct2/world/Footpath.cpp`
- **Footpath header**: `src/openrct2/world/Footpath.h`
- **Path element definition**: `src/openrct2/world/tile_element/PathElement.h`
- **Direction constants**: `src/openrct2/core/Direction.hpp`
- **Tests**: `test/tests/Pathfinding.cpp`

## Testing

### Unit Tests

```bash
cd bin
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DWITH_TESTS=on
ninja
./openrct2-cli scan-objects
ctest -j 2 --output-on-failure
```

The pathfinding tests use a test park at `bin/testdata/pathfinding-tests.sv6` with predefined scenarios.

### Manual Testing

1. Build paths with various configurations (straight, curved, slopes, bridges)
2. Place ride entrances at different locations
3. Give guests maps (or use "leaving park" guests)
4. Observe that guests reach their destinations without:
   - Walking in circles
   - Getting stuck at dead ends
   - Walking away from reachable goals
   - Ignoring obvious shorter paths

### Debug Logging

In `GuestPathfinding.cpp`, set `kLogPathfinding = true` and add `PEEP_FLAGS_DEBUG_PATHFINDING` to a guest to enable verbose pathfinding output.

## Integration Point

The A* implementation should be called from `GuestPathfinding::ChooseDirection` when conditions are appropriate (guest has map, guest is leaving park, etc.). The function should return the first direction to walk, or `kInvalidDirection` on failure.

```cpp
// In ChooseDirection, when A* is appropriate:
Direction astarResult = AStarChooseDirection(currentPos, goal, guest, availableEdges);
if (astarResult != kInvalidDirection) {
    return astarResult;
}
// Fall back to DFS or random wandering
```
