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

## Transport Shortcuts

Guests with maps can use transport rides (chairlifts, monorails, trains, etc.) as shortcuts to reach their destination faster.

### How It Works

1. **Detection**: When `CalculateNextDestination()` runs, it calls `ShouldUseTransportRide()` to check if any transport ride would get the guest closer to their goal
2. **Cost Calculation**: Compares walking distance to goal vs. (walk to entrance + boarding cost + walk from exit to goal)
3. **Shortcut Activation**: If transport saves enough distance, sets `PEEP_FLAGS_TRANSPORT_SHORTCUT` and redirects guest to the transport ride entrance
4. **Boarding**: Guest skips intensity/nausea checks when boarding (they're using it for transport, not thrills)
5. **Completion**: After exiting at destination station, guest continues to original goal

### Key State

| Field | Purpose |
|-------|---------|
| `PEEP_FLAGS_TRANSPORT_SHORTCUT` | Flag indicating guest is using transport to reach destination |
| `GuestTransportDestination` | Original destination ride (stored while heading to transport) |
| `GuestHeadingToRideId` | Currently points to transport ride while shortcut is active |
| `GuestRejectedTransport` | Transport ride guest balked at (prevents immediate retry) |
| `GuestRejectedTransportGoal` | Goal at time of rejection (allows retry if goal changes) |

### Key Functions

| Function | Location | Purpose |
|----------|----------|---------|
| `ShouldUseTransportRide()` | GuestPathfinding.cpp | Evaluates if transport would help |
| `FindBestTransportOption()` | GuestPathfinding.cpp | Finds best transport ride and stations |
| `skipRideChecks` logic | Guest.cpp | Skips intensity checks for transport shortcuts |
| `GuestTriedToEnterFullQueue()` | Guest.cpp | Restores original destination if queue full |

### Edge Cases Handled

- **Queue full**: Guest's original destination is restored (not made aimless)
- **Destination override**: `PeepHeadForNearestRide()` won't override destination while shortcut is active
- **Free transport only**: Only considers free transport rides (paid rides not used as shortcuts)
- **Rejection tracking**: Won't retry same transport for same goal (prevents loops)

## Debugging

### Guest Debug View (Recommended)

Enable `debugging_tools` in settings, then open a guest window and click the debug tab (gear icon). Shows:

| Field | Description |
|-------|-------------|
| Entity ID | Internal entity identifier |
| Position | Current world coordinates |
| Next | Next tile location |
| Dest | Immediate walking destination |
| Pathfind Goal | Target tile for DFS (may be stale for A* users) |
| **Heading to Ride** | The ride `GuestHeadingToRideId` points to |
| **Has Map** | "Yes (using A*)", "Yes", or "No" - shows which algorithm is active |
| **Transport Shortcut** | "None" or "Active (Ride Name)" |
| **Original Goal** | Destination before transport shortcut was set |
| Pathfind History | Recent junction decisions (DFS only) |

The "Has Map" field explains why "Pathfind Goal" might look wrong - A* users don't update that field.

### Console Logging (Disabled by Default)

For detailed console output, enable logging in the source files:

**A* Logging** (`AStarPathfinding.cpp`):
```cpp
static constexpr bool kLogAStarPathfinding = true;  // Change to true
static constexpr const char* kLogAStarPeepName = "Pete Pathfinder";
```

**DFS/Transport Logging** (`GuestPathfinding.cpp`):
```cpp
static constexpr bool kLogPathfinding = true;  // Change to true
static constexpr const char* kLogPathfindingPeepName = "Pete Pathfinder";
```

To use: rename a guest to "Pete Pathfinder", give them a map, and watch console output for `[A*]` or `[Pathfinding]` messages.

## Line Reference

| Concept | File | Lines |
|---------|------|-------|
| CalculateNextDestination (entry point) | GuestPathfinding.cpp | ~2290+ |
| ChooseDirection | GuestPathfinding.cpp | ~1276-1620 |
| A* algorithm | AStarPathfinding.cpp | ~100-350 |
| DFS heuristic search | GuestPathfinding.cpp | ~766-1267 |
| Aimless walking | GuestPathfinding.cpp | ~583-602 |
| ShouldUseTransportRide | GuestPathfinding.cpp | ~2130-2290 |
| FindBestTransportOption | GuestPathfinding.cpp | ~2130-2290 |
| Transport shortcut activation | GuestPathfinding.cpp | ~2350-2380 |
| skipRideChecks (transport boarding) | Guest.cpp | ~2060-2070 |
| GuestTriedToEnterFullQueue | Guest.cpp | ~2500-2530 |
| PeepHeadForNearestRide (shortcut guard) | Guest.cpp | ~3290-3310 |
| IsInWidePathArea | GuestPathfinding.cpp | ~205-238 |
| Queue handling | GuestPathfinding.cpp | ~927-937 |
| Guest debug view rendering | windows/Guest.cpp | ~1885-1960 |

*Note: Line numbers are approximate and may shift as code changes.*

## Testing

```
powershell.exe -Command "cd 'N:\src\OpenRCT2\bin'; .\tests.exe --gtest_filter=*Pathfinding*"
```

Test file: `test/tests/Pathfinding.cpp`

Tests cover:
- Basic pathfinding (straight, curved, sloped paths)
- Impossible paths (gaps, fences, cliffs)
- A* pathfinding for guests with maps
- Transport ride selection logic
