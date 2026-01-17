# Guest Pathfinding System

## Intended Behavior

**Guests with maps** (heading to a ride or exit) should find efficient paths and walk directly across wide path areas (plazas) to their destination.

**Guests without maps** should find reasonable paths but avoid wide areas - they stick to the "skeleton" of thin paths since the DFS algorithm gets confused in open plazas.

**Aimless guests** (no destination) wander randomly with a slight tendency to continue in their current direction.

**All guests** should avoid walking down dead ends, through ride exit areas, or cutting through other rides' queue lines.

## Source Files

| File | Purpose |
|------|---------|
| `src/openrct2/peep/GuestPathfinding.cpp` | Main logic, DFS, aimless walking, edge culling |
| `src/openrct2/peep/AStarPathfinding.cpp` | A* for map holders and guests leaving park |
| `src/openrct2/entity/Guest.cpp` | Helper functions like `HeadingForRideOrParkExit()` |

## Algorithm Selection

| Guest State | Algorithm | Why |
|-------------|-----------|-----|
| Has map + heading somewhere | A* | Optimal paths, handles wide areas |
| Leaving park | A* | Needs to find exit efficiently |
| Heading to ride (no map) | DFS Heuristic | Good enough, avoids wide area confusion |
| No destination (aimless) | Random walk | Just wandering |
| Off paths (on grass) | Surface pathfinding | Wall collision avoidance |

The key entry point is `CalculateNextDestination()` which dispatches to the appropriate algorithm.

## Wide Path Handling

This is the most complex part of the system.

**The problem:** Wide paths (2x2+ connected areas like plazas) have many edges at every tile. DFS counts junctions and gets overwhelmed. A* handles them fine with its heuristic.

**The solution:**
- A* users traverse wide paths directly
- DFS users have wide path edges filtered out, forcing them onto the "skeleton" (the thin path network running through/around the plaza)
- `IsWide()` flag marks tiles that are part of a wide area
- `IsInWidePathArea()` detects if a tile is in a 2x2+ connected area (used to skip map reading animations in plazas)

**Key invariant:** The `willUseAStar` check must verify the guest is actually heading somewhere, not just that they have a map. Aimless guests with maps use random walking, not A*.

## Edge Culling

Before pathfinding, "bad" edges are removed from consideration:

| Edge Type | Culled? | Why |
|-----------|---------|-----|
| Dead ends | Yes | Don't walk into corners |
| Ride exits | Yes | Exit areas aren't for walking |
| Wide paths | Only for DFS users | A* handles them, DFS doesn't |

Culling is skipped ~34% of the time (randomness) and for guests carrying food/drink (they're distracted).

## Queue Handling

Guests cannot cut through queue lines for other rides. The `ignoreForeignQueues` flag blocks paths through queues unless it's the queue for their destination ride.

## Debugging

### Pete Pathfinder (A* Logging)

`AStarPathfinding.cpp` has built-in logging filtered by guest name:

```cpp
static constexpr bool kLogAStarPathfinding = true;
static constexpr const char* kLogAStarPeepName = "Pete Pathfinder";
```

To debug: rename a guest to "Pete Pathfinder", give them a map, and watch console output for `[A*]` messages showing start/goal positions, tiles explored, and direction chosen.

### DFS Logging

`LogPathfinding()` in `GuestPathfinding.cpp` provides similar tracing for DFS.

## Line Reference

| Concept | File | Lines |
|---------|------|-------|
| CalculateNextDestination (entry point) | GuestPathfinding.cpp | 1940-2196 |
| ChooseDirection | GuestPathfinding.cpp | 1276-1620 |
| A* algorithm | AStarPathfinding.cpp | 294-507 |
| DFS heuristic search | GuestPathfinding.cpp | 766-1267 |
| Aimless walking | GuestPathfinding.cpp | 583-602 |
| willUseAStar check | GuestPathfinding.cpp | 1965-1969 |
| Wide path filtering | GuestPathfinding.cpp | 1971-1990 |
| Edge culling | GuestPathfinding.cpp | 2041-2069 |
| IsInWidePathArea | GuestPathfinding.cpp | 205-238 |
| Queue handling | GuestPathfinding.cpp | 927-937 |
| Map reading | GuestPathfinding.cpp | 2076-2091 |
| Park entrance leaving | GuestPathfinding.cpp | 1723-1752 |

## Testing

```powershell
.\bin\tests.exe --gtest_filter=*Pathfinding*
```

Test file: `test/tests/Pathfinding.cpp`
