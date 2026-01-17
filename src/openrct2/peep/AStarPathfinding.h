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
#include "../world/Location.hpp"

struct Peep;

namespace OpenRCT2::PathFinding
{
    Direction AStarChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, const Peep& peep, bool ignoreForeignQueues,
        RideId queueRideIndex);
}
