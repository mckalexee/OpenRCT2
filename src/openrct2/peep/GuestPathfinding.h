/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../ride/RideTypes.h"
#include "../ride/Station.h"
#include "../world/Location.hpp"

#include <memory>
#include <utility>

struct Peep;
struct Guest;

namespace OpenRCT2
{
    struct TileElement;
}

namespace OpenRCT2::PathFinding
{
    Direction ChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, Peep& peep, bool ignoreForeignQueues, RideId queueRideIndex);

    int32_t CalculateNextDestination(Guest& peep);

    int32_t GuestPathFindParkEntranceEntering(Peep& peep, uint8_t edges);

    int32_t GuestPathFindPeepSpawn(Peep& peep, uint8_t edges);

    int32_t GuestPathFindParkEntranceLeaving(Peep& peep, uint8_t edges);

    // Transport ride selection - exposed for testing
    std::pair<RideId, StationIndex> ShouldUseTransportRide(
        const Guest& guest, const TileCoordsXYZ& currentPos, const TileCoordsXYZ& goalPos);

    // Invalidate cached list of transport rides (call when rides are added/removed)
    void InvalidateTransportRideCache();

} // namespace OpenRCT2::PathFinding
