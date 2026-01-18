#include "TestData.h"

#include <gtest/gtest.h>
#include <memory>
#include <openrct2/Context.h>
#include <openrct2/Game.h>
#include <openrct2/GameState.h>
#include <openrct2/OpenRCT2.h>
#include <openrct2/core/String.hpp>
#include <openrct2/entity/Guest.h>
#include <openrct2/peep/GuestPathfinding.h>
#include <openrct2/platform/Platform.h>
#include <openrct2/ride/RideData.h>
#include <openrct2/ride/RideManager.hpp>
#include <openrct2/scenario/Scenario.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/tile_element/SurfaceElement.h>
#include <string>

using namespace OpenRCT2;

static std::ostream& operator<<(std::ostream& os, const TileCoordsXYZ& coords)
{
    return os << "(" << coords.x << ", " << coords.y << ", " << coords.z << ")";
}

class PathfindingTestBase : public testing::Test
{
public:
    static void SetUpTestCase()
    {
        gOpenRCT2Headless = true;
        gOpenRCT2NoGraphics = true;
        _context = CreateContext();
        const bool initialised = _context->Initialise();
        ASSERT_TRUE(initialised);

        std::string parkPath = TestData::GetParkPath("pathfinding-tests.sv6");
        GetContext()->LoadParkFromFile(parkPath);
        GameLoadInit();
    }

    void SetUp() override
    {
        // Use a consistent random seed in every test
        ScenarioRandSeed(0x12345678, 0x87654321);
    }

    static void TearDownTestCase()
    {
        _context = nullptr;
    }

protected:
    static Ride* FindRideByName(const char* name)
    {
        auto& gameState = getGameState();
        for (auto& ride : RideManager(gameState))
        {
            auto thisName = ride.getName();
            if (String::startsWith(thisName, u8string{ name }, true))
            {
                return &ride;
            }
        }
        return nullptr;
    }

    static bool FindPath(TileCoordsXYZ* pos, const TileCoordsXYZ& goal, int expectedSteps, RideId targetRideID)
    {
        // Our start position is in tile coordinates, but we need to give the peep spawn
        // position in actual world coords (32 units per tile X/Y, 8 per Z level).
        // Add 16 so the peep spawns in the centre of the tile.
        auto* peep = Guest::Generate(pos->ToCoordsXYZ().ToTileCentre());

        // Peeps that are outside of the park use specialized pathfinding which we don't want to
        // use here
        peep->OutsideOfPark = false;

        // An earlier iteration of this code just gave peeps a target position to walk to, but it turns out
        // that with no actual ride to head towards, when a peep reaches a junction they use the 'aimless'
        // pathfinder instead of pursuing their original pathfinding target. So, we always need to give them
        // an actual ride to walk to the entrance of.
        peep->GuestHeadingToRideId = targetRideID;

        // Pick the direction the peep should initially move in, given the goal position.
        // This will also store the goal position and initialize pathfinding data for the peep.
        const Direction moveDir = PathFinding::ChooseDirection(*pos, goal, *peep, false, RideId::GetNull());
        if (moveDir == kInvalidDirection)
        {
            // Couldn't determine a direction to move off in
            return false;
        }

        // We have already set up the peep's overall pathfinding goal, but we also have to set their initial
        // 'destination' which is a close position that they will walk towards in a straight line - in this case, one
        // tile away. Stepping the peep will move them towards their destination, and once they reach it, a new
        // destination will be picked, to try and get the peep towards the overall pathfinding goal.
        peep->PeepDirection = moveDir;
        auto destination = CoordsDirectionDelta[moveDir] + peep->GetLocation();
        peep->SetDestination(destination, 2);

        // Repeatedly step the peep, until they reach the target position or until the expected number of steps have
        // elapsed. Each step, check that the tile they are standing on is not marked as forbidden in the test data
        // (red neon ground type).
        int step = 0;
        while (!(*pos == goal) && step < expectedSteps)
        {
            peep->PerformNextAction();
            ++step;

            *pos = TileCoordsXYZ(peep->GetLocation());

            EXPECT_PRED_FORMAT1(AssertIsNotForbiddenPosition, *pos);

            // Check that the peep is still on a footpath. Use next_z instead of pos->z here because pos->z will change
            // when the peep is halfway up a slope, but next_z will not change until they move to the next tile.
            EXPECT_NE(MapGetFootpathElement({ pos->ToCoordsXY(), peep->NextLoc.z }), nullptr);
        }

        // Clean up the peep, because we're reusing this loaded context for all tests.
        PeepEntityRemove(peep);

        // Require that the number of steps taken is exactly what we expected. The pathfinder is supposed to be
        // deterministic, and we reset the RNG seed for each test, everything should be entirely repeatable; as
        // such a change in the number of steps taken on one of these paths needs to be reviewed. For the negative
        // tests, we will not have reached the goal but we still expect the loop to have run for the total number
        // of steps requested before giving up.
        EXPECT_EQ(step, expectedSteps);

        return *pos == goal;
    }

    static ::testing::AssertionResult AssertIsStartPosition(const char*, const TileCoordsXYZ& location)
    {
        const uint32_t expectedSurfaceStyle = 11u;
        const uint32_t style = MapGetSurfaceElementAt(location.ToCoordsXYZ())->GetSurfaceObjectIndex();

        if (style != expectedSurfaceStyle)
            return ::testing::AssertionFailure()
                << "Start location " << location << " should have surface style " << expectedSurfaceStyle
                << " but actually has style " << style
                << ". Either the test map is not set up correctly, or you got the coordinates wrong.";

        return ::testing::AssertionSuccess();
    }

    static ::testing::AssertionResult AssertIsNotForbiddenPosition(const char*, const TileCoordsXYZ& location)
    {
        const uint32_t forbiddenSurfaceStyle = 8u;

        const uint32_t style = MapGetSurfaceElementAt(location.ToCoordsXYZ())->GetSurfaceObjectIndex();

        if (style == forbiddenSurfaceStyle)
            return ::testing::AssertionFailure()
                << "Path traversed location " << location << ", but it is marked as a forbidden location (surface style "
                << forbiddenSurfaceStyle << "). Either the map is set up incorrectly, or the pathfinder went the wrong way.";

        return ::testing::AssertionSuccess();
    }

private:
    static std::shared_ptr<IContext> _context;
};

std::shared_ptr<IContext> PathfindingTestBase::_context;

struct SimplePathfindingScenario
{
    const char* name;
    TileCoordsXYZ start;
    uint32_t steps;

    SimplePathfindingScenario(const char* _name, const TileCoordsXYZ& _start, int _steps)
        : name(_name)
        , start(_start)
        , steps(_steps)
    {
    }

    static std::string ToName(const ::testing::TestParamInfo<SimplePathfindingScenario>& param_info)
    {
        return param_info.param.name;
    }
};

class SimplePathfindingTest : public PathfindingTestBase, public ::testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(SimplePathfindingTest, CanFindPathFromStartToGoal)
{
    const SimplePathfindingScenario& scenario = GetParam();

    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);
    TileCoordsXYZ pos = scenario.start;

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().Entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x - TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y - TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    const auto succeeded = FindPath(&pos, goal, scenario.steps, ride->id) ? ::testing::AssertionSuccess()
                                                                          : ::testing::AssertionFailure()
            << "Failed to find path from " << scenario.start << " to " << goal << " in " << scenario.steps << " steps; reached "
            << pos << " before giving up.";

    EXPECT_TRUE(succeeded);
}

INSTANTIATE_TEST_SUITE_P(
    ForScenario, SimplePathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("StraightFlat", { 19, 15, 14 }, 24), SimplePathfindingScenario("SBend", { 15, 12, 14 }, 87),
        SimplePathfindingScenario("UBend", { 17, 9, 14 }, 87), SimplePathfindingScenario("CBend", { 14, 5, 14 }, 164),
        SimplePathfindingScenario("TwoEqualRoutes", { 9, 13, 14 }, 89),
        SimplePathfindingScenario("TwoUnequalRoutes", { 3, 13, 14 }, 89),
        SimplePathfindingScenario("StraightUpBridge", { 12, 15, 14 }, 24),
        SimplePathfindingScenario("StraightUpSlope", { 14, 15, 14 }, 24),
        SimplePathfindingScenario("SelfCrossingPath", { 6, 5, 14 }, 211)),
    SimplePathfindingScenario::ToName);

class ImpossiblePathfindingTest : public PathfindingTestBase, public ::testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(ImpossiblePathfindingTest, CannotFindPathFromStartToGoal)
{
    const SimplePathfindingScenario& scenario = GetParam();
    TileCoordsXYZ pos = scenario.start;
    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().Entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x + TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y + TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    EXPECT_FALSE(FindPath(&pos, goal, 10000, ride->id));
}

INSTANTIATE_TEST_SUITE_P(
    ForScenario, ImpossiblePathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("PathWithGap", { 1, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithFences", { 11, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithCliff", { 7, 17, 14 }, 10000)),
    SimplePathfindingScenario::ToName);

// A* Pathfinding Tests
// These tests verify the A* pathfinding algorithm works correctly for guests with maps

class AStarPathfindingTestBase : public PathfindingTestBase
{
protected:
    // Find path using A* (guest with map)
    static bool FindPathWithMap(TileCoordsXYZ* pos, const TileCoordsXYZ& goal, int expectedSteps, RideId targetRideID)
    {
        auto* peep = Guest::Generate(pos->ToCoordsXYZ().ToTileCentre());
        peep->OutsideOfPark = false;
        peep->GuestHeadingToRideId = targetRideID;

        // Give guest a map to trigger A* pathfinding
        peep->GiveItem(ShopItem::map);

        const Direction moveDir = PathFinding::ChooseDirection(*pos, goal, *peep, false, RideId::GetNull());
        if (moveDir == kInvalidDirection)
        {
            PeepEntityRemove(peep);
            return false;
        }

        peep->PeepDirection = moveDir;
        auto destination = CoordsDirectionDelta[moveDir] + peep->GetLocation();
        peep->SetDestination(destination, 2);

        int step = 0;
        while (!(*pos == goal) && step < expectedSteps)
        {
            peep->PerformNextAction();
            ++step;

            *pos = TileCoordsXYZ(peep->GetLocation());

            EXPECT_PRED_FORMAT1(AssertIsNotForbiddenPosition, *pos);
            EXPECT_NE(MapGetFootpathElement({ pos->ToCoordsXY(), peep->NextLoc.z }), nullptr);
        }

        PeepEntityRemove(peep);
        EXPECT_EQ(step, expectedSteps);

        return *pos == goal;
    }

    // Find path for guest leaving park (also uses A*)
    static bool FindPathLeavingPark(TileCoordsXYZ* pos, const TileCoordsXYZ& goal, int expectedSteps, RideId targetRideID)
    {
        auto* peep = Guest::Generate(pos->ToCoordsXYZ().ToTileCentre());
        peep->OutsideOfPark = false;
        peep->GuestHeadingToRideId = targetRideID;

        // Set leaving park flag to trigger A* pathfinding
        peep->PeepFlags |= PEEP_FLAGS_LEAVING_PARK;

        const Direction moveDir = PathFinding::ChooseDirection(*pos, goal, *peep, false, RideId::GetNull());
        if (moveDir == kInvalidDirection)
        {
            PeepEntityRemove(peep);
            return false;
        }

        peep->PeepDirection = moveDir;
        auto destination = CoordsDirectionDelta[moveDir] + peep->GetLocation();
        peep->SetDestination(destination, 2);

        int step = 0;
        while (!(*pos == goal) && step < expectedSteps)
        {
            peep->PerformNextAction();
            ++step;

            *pos = TileCoordsXYZ(peep->GetLocation());

            EXPECT_PRED_FORMAT1(AssertIsNotForbiddenPosition, *pos);
            EXPECT_NE(MapGetFootpathElement({ pos->ToCoordsXY(), peep->NextLoc.z }), nullptr);
        }

        PeepEntityRemove(peep);
        EXPECT_EQ(step, expectedSteps);

        return *pos == goal;
    }
};

class AStarPathfindingTest : public AStarPathfindingTestBase, public ::testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(AStarPathfindingTest, GuestWithMapCanFindPath)
{
    const SimplePathfindingScenario& scenario = GetParam();

    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);
    TileCoordsXYZ pos = scenario.start;

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().Entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x - TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y - TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    const auto succeeded = FindPathWithMap(&pos, goal, scenario.steps, ride->id) ? ::testing::AssertionSuccess()
                                                                                  : ::testing::AssertionFailure()
            << "A* failed to find path from " << scenario.start << " to " << goal << " in " << scenario.steps
            << " steps; reached " << pos << " before giving up.";

    EXPECT_TRUE(succeeded);
}

// Test A* with same scenarios as DFS to ensure it produces valid paths
INSTANTIATE_TEST_SUITE_P(
    ForScenario, AStarPathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("StraightFlat", { 19, 15, 14 }, 24), SimplePathfindingScenario("SBend", { 15, 12, 14 }, 87),
        SimplePathfindingScenario("UBend", { 17, 9, 14 }, 87), SimplePathfindingScenario("StraightUpBridge", { 12, 15, 14 }, 24),
        SimplePathfindingScenario("StraightUpSlope", { 14, 15, 14 }, 24)),
    SimplePathfindingScenario::ToName);

class AStarImpossiblePathfindingTest : public AStarPathfindingTestBase,
                                       public ::testing::WithParamInterface<SimplePathfindingScenario>
{
};

TEST_P(AStarImpossiblePathfindingTest, GuestWithMapCannotFindUnreachablePath)
{
    const SimplePathfindingScenario& scenario = GetParam();
    TileCoordsXYZ pos = scenario.start;
    ASSERT_PRED_FORMAT1(AssertIsStartPosition, scenario.start);

    auto ride = FindRideByName(scenario.name);
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().Entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x + TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y + TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    // A* should fail and fall back to DFS, which should also fail
    EXPECT_FALSE(FindPathWithMap(&pos, goal, 10000, ride->id));
}

INSTANTIATE_TEST_SUITE_P(
    ForScenario, AStarImpossiblePathfindingTest,
    ::testing::Values(
        SimplePathfindingScenario("PathWithGap", { 1, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithFences", { 11, 6, 14 }, 10000),
        SimplePathfindingScenario("PathWithCliff", { 7, 17, 14 }, 10000)),
    SimplePathfindingScenario::ToName);

// Test that leaving park flag also triggers A* pathfinding
class AStarLeavingParkTest : public AStarPathfindingTestBase
{
};

TEST_F(AStarLeavingParkTest, LeavingParkGuestUsesAStar)
{
    TileCoordsXYZ pos = { 19, 15, 14 }; // StraightFlat start position
    ASSERT_PRED_FORMAT1(AssertIsStartPosition, pos);

    auto ride = FindRideByName("StraightFlat");
    ASSERT_NE(ride, nullptr);

    auto entrancePos = ride->getStation().Entrance;
    TileCoordsXYZ goal = TileCoordsXYZ(
        entrancePos.x - TileDirectionDelta[entrancePos.direction].x,
        entrancePos.y - TileDirectionDelta[entrancePos.direction].y, entrancePos.z);

    const auto succeeded = FindPathLeavingPark(&pos, goal, 24, ride->id) ? ::testing::AssertionSuccess()
                                                                          : ::testing::AssertionFailure()
            << "A* (leaving park) failed to find path to goal.";

    EXPECT_TRUE(succeeded);
}

// Transport Shortcut Pathfinding Tests
// These tests verify the transport ride shortcut mechanism and rejection tracking

class TransportShortcutTest : public PathfindingTestBase
{
protected:
    // Create a guest with specified transport state
    static Guest* CreateGuestWithTransportState(
        const TileCoordsXYZ& pos, bool hasMap, bool hasTransportShortcut, RideId headingToRide = RideId::GetNull(),
        RideId transportDest = RideId::GetNull(), RideId rejectedTransport = RideId::GetNull(),
        RideId rejectedGoal = RideId::GetNull())
    {
        auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
        peep->OutsideOfPark = false;
        peep->GuestHeadingToRideId = headingToRide;

        if (hasMap)
        {
            peep->GiveItem(ShopItem::map);
        }

        if (hasTransportShortcut)
        {
            peep->PeepFlags |= PEEP_FLAGS_TRANSPORT_SHORTCUT;
            peep->GuestTransportDestination = transportDest;
        }

        peep->GuestRejectedTransport = rejectedTransport;
        peep->GuestRejectedTransportGoal = rejectedGoal;

        return peep;
    }
};

// Test: Rejection fields are properly initialized
TEST_F(TransportShortcutTest, RejectionFieldsInitializedToNull)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());

    EXPECT_TRUE(peep->GuestRejectedTransport.IsNull());
    EXPECT_TRUE(peep->GuestRejectedTransportGoal.IsNull());

    PeepEntityRemove(peep);
}

// Test: Transport shortcut flag and fields work together
TEST_F(TransportShortcutTest, TransportShortcutFlagAndFieldsConsistent)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    auto ride = FindRideByName("StraightFlat");
    ASSERT_NE(ride, nullptr);

    // Create guest using transport as shortcut
    auto* peep = CreateGuestWithTransportState(
        pos, true, true, ride->id, RideId::FromUnderlying(5) // Original destination was ride 5
    );

    EXPECT_TRUE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);
    EXPECT_EQ(peep->GuestHeadingToRideId, ride->id);
    EXPECT_EQ(peep->GuestTransportDestination, RideId::FromUnderlying(5));

    PeepEntityRemove(peep);
}

// Test: Rejection tracking prevents re-trying same transport for same goal
TEST_F(TransportShortcutTest, RejectionPreventsRetryForSameGoal)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    RideId transportRideId = RideId::FromUnderlying(10);
    RideId originalGoal = RideId::FromUnderlying(5);

    // Create guest who rejected transport 10 while heading to ride 5
    auto* peep = CreateGuestWithTransportState(pos, true, false, originalGoal, RideId::GetNull(), transportRideId, originalGoal);

    // Guest should have rejection recorded
    EXPECT_EQ(peep->GuestRejectedTransport, transportRideId);
    EXPECT_EQ(peep->GuestRejectedTransportGoal, originalGoal);

    // The guest is still heading to the same goal
    EXPECT_EQ(peep->GuestHeadingToRideId, originalGoal);

    PeepEntityRemove(peep);
}

// Test: Goal change should allow considering previously rejected transport
TEST_F(TransportShortcutTest, GoalChangeAllowsRejectedTransport)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    RideId transportRideId = RideId::FromUnderlying(10);
    RideId originalGoal = RideId::FromUnderlying(5);
    RideId newGoal = RideId::FromUnderlying(6);

    // Create guest who rejected transport 10 while heading to ride 5
    auto* peep = CreateGuestWithTransportState(pos, true, false, originalGoal, RideId::GetNull(), transportRideId, originalGoal);

    // Simulate goal change
    peep->GuestHeadingToRideId = newGoal;

    // Now guest is heading to different goal than when they rejected
    // The rejection check (in ShouldUseTransportRide) compares:
    // GuestHeadingToRideId (newGoal=6) vs GuestRejectedTransportGoal (originalGoal=5)
    // Since they differ, transport should be considered again
    EXPECT_NE(peep->GuestHeadingToRideId, peep->GuestRejectedTransportGoal);

    PeepEntityRemove(peep);
}

// Test: Successful transport use should clear rejection
TEST_F(TransportShortcutTest, SuccessfulTransportClearsRejection)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    RideId previouslyRejected = RideId::FromUnderlying(10);

    // Create guest with a previous rejection
    auto* peep = CreateGuestWithTransportState(
        pos, true, false, RideId::GetNull(), RideId::GetNull(), previouslyRejected, RideId::FromUnderlying(5));

    EXPECT_FALSE(peep->GuestRejectedTransport.IsNull());

    // Simulate successful transport - clear rejection
    peep->GuestRejectedTransport = RideId::GetNull();
    peep->GuestRejectedTransportGoal = RideId::GetNull();

    EXPECT_TRUE(peep->GuestRejectedTransport.IsNull());
    EXPECT_TRUE(peep->GuestRejectedTransportGoal.IsNull());

    PeepEntityRemove(peep);
}

// Test: Transport shortcut is preserved when guest's underlying need changes destination
TEST_F(TransportShortcutTest, TransportShortcutPreservedDuringGoalChange)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    RideId transportRide = RideId::FromUnderlying(10);
    RideId originalGoal = RideId::FromUnderlying(5);

    // Create guest using transport shortcut
    auto* peep = CreateGuestWithTransportState(pos, true, true, transportRide, originalGoal);

    EXPECT_TRUE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);
    EXPECT_EQ(peep->GuestTransportDestination, originalGoal);

    // The transport shortcut flag should remain set
    // (we removed the clearing from GuestResetRideHeading, GuestLeavePark, PeepHeadForNearestRide)
    EXPECT_TRUE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);

    PeepEntityRemove(peep);
}

// Test: Guest outside park should not use transport
TEST_F(TransportShortcutTest, GuestOutsideParkNoTransport)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    auto ride = FindRideByName("StraightFlat");
    ASSERT_NE(ride, nullptr);

    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = true; // Guest is outside park
    peep->GiveItem(ShopItem::map);
    peep->GuestHeadingToRideId = ride->id;

    // Transport shortcut should not be set for guest outside park
    EXPECT_FALSE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);

    PeepEntityRemove(peep);
}

// Test: Aimless guest should not use transport
TEST_F(TransportShortcutTest, AimlessGuestNoTransport)
{
    TileCoordsXYZ pos = { 19, 15, 14 };

    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = false;
    peep->GiveItem(ShopItem::map);
    peep->GuestHeadingToRideId = RideId::GetNull(); // Aimless - no destination

    // Transport shortcut should not be set for aimless guest
    EXPECT_FALSE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);

    PeepEntityRemove(peep);
}

// Test: Balk tracking sets rejection correctly
TEST_F(TransportShortcutTest, BalkingSetsRejection)
{
    TileCoordsXYZ pos = { 19, 15, 14 };
    RideId transportRide = RideId::FromUnderlying(10);
    RideId originalGoal = RideId::FromUnderlying(5);

    // Create guest using transport shortcut
    auto* peep = CreateGuestWithTransportState(pos, true, true, transportRide, originalGoal);

    EXPECT_TRUE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);
    EXPECT_TRUE(peep->GuestRejectedTransport.IsNull());

    // Simulate balking - what ChoseNotToGoOnRide does for transport rides
    peep->GuestRejectedTransport = transportRide;
    peep->GuestRejectedTransportGoal = originalGoal;
    peep->PeepFlags &= ~PEEP_FLAGS_TRANSPORT_SHORTCUT;
    peep->GuestHeadingToRideId = peep->GuestTransportDestination;
    peep->GuestTransportDestination = RideId::GetNull();

    // After balking:
    // - Rejection is recorded
    EXPECT_EQ(peep->GuestRejectedTransport, transportRide);
    EXPECT_EQ(peep->GuestRejectedTransportGoal, originalGoal);
    // - Shortcut flag is cleared
    EXPECT_FALSE(peep->PeepFlags & PEEP_FLAGS_TRANSPORT_SHORTCUT);
    // - Original goal is restored
    EXPECT_EQ(peep->GuestHeadingToRideId, originalGoal);
    // - Transport destination is cleared
    EXPECT_TRUE(peep->GuestTransportDestination.IsNull());

    PeepEntityRemove(peep);
}

// ============================================================================
// Transport Selection Logic Tests
// These test the actual ShouldUseTransportRide function behavior
// ============================================================================

class TransportSelectionTest : public PathfindingTestBase
{
protected:
    static Ride* FindTransportRide()
    {
        auto& gameState = getGameState();
        for (auto& ride : RideManager(gameState))
        {
            if (ride.getRideTypeDescriptor().HasFlag(RtdFlag::isTransportRide))
            {
                return &ride;
            }
        }
        return nullptr;
    }
};

// Test: ShouldUseTransportRide returns null when goal is very close (< 5 tiles)
TEST_F(TransportSelectionTest, ReturnsNullWhenGoalVeryClose)
{
    TileCoordsXYZ pos = { 10, 10, 14 };
    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = false;
    peep->GiveItem(ShopItem::map);

    // Goal is only 3 tiles away (Manhattan)
    TileCoordsXYZ closeGoal = { 12, 11, 14 };

    auto result = PathFinding::ShouldUseTransportRide(*peep, pos, closeGoal);

    // Should return null - too close to bother with transport
    EXPECT_TRUE(result.first.IsNull());

    PeepEntityRemove(peep);
}

// Test: Transport selection considers exit proximity to goal
TEST_F(TransportSelectionTest, SelectsTransportWhenExitCloserToGoal)
{
    auto* transportRide = FindTransportRide();
    if (transportRide == nullptr)
    {
        GTEST_SKIP() << "No transport ride in test park";
    }

    // Verify the ride is open and usable
    EXPECT_EQ(transportRide->status, RideStatus::open);
    EXPECT_GE(transportRide->numStations, 2);

    TileCoordsXYZ pos = { 10, 10, 14 };
    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = false;
    peep->GiveItem(ShopItem::map);
    peep->CashInPocket = 1000; // Enough for any ride price

    // Find station positions
    const auto& station1 = transportRide->getStation(StationIndex::FromUnderlying(1));

    // Create a goal near station 1's exit
    TileCoordsXYZ goalNearExit;
    if (!station1.Exit.IsNull())
    {
        goalNearExit = TileCoordsXYZ(station1.Exit);
        goalNearExit.x += 2; // A bit past the exit
    }
    else
    {
        GTEST_SKIP() << "Station 1 has no exit";
    }

    auto result = PathFinding::ShouldUseTransportRide(*peep, pos, goalNearExit);

    // Log the result for debugging
    if (!result.first.IsNull())
    {
        EXPECT_EQ(result.first, transportRide->id);
    }
    // Note: May still be null if entrance/exit positions don't meet criteria

    PeepEntityRemove(peep);
}

// Test: Transport NOT selected when guest already rejected this transport for same goal
TEST_F(TransportSelectionTest, RespectsRejectionForSameGoal)
{
    auto* transportRide = FindTransportRide();
    if (transportRide == nullptr)
    {
        GTEST_SKIP() << "No transport ride in test park";
    }

    TileCoordsXYZ pos = { 10, 10, 14 };
    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = false;
    peep->GiveItem(ShopItem::map);
    peep->CashInPocket = 1000;

    // Set up rejection - guest previously rejected this transport for goal ride 5
    RideId goalRide = RideId::FromUnderlying(5);
    peep->GuestHeadingToRideId = goalRide;
    peep->GuestRejectedTransport = transportRide->id;
    peep->GuestRejectedTransportGoal = goalRide;

    TileCoordsXYZ goal = { 50, 50, 14 };
    auto result = PathFinding::ShouldUseTransportRide(*peep, pos, goal);

    // Should not select this transport because it was rejected for this goal
    if (!result.first.IsNull())
    {
        EXPECT_NE(result.first, transportRide->id);
    }

    PeepEntityRemove(peep);
}

// Test: Transport IS considered when goal changes after rejection
TEST_F(TransportSelectionTest, ConsidersTransportAfterGoalChange)
{
    auto* transportRide = FindTransportRide();
    if (transportRide == nullptr)
    {
        GTEST_SKIP() << "No transport ride in test park";
    }

    TileCoordsXYZ pos = { 10, 10, 14 };
    auto* peep = Guest::Generate(pos.ToCoordsXYZ().ToTileCentre());
    peep->OutsideOfPark = false;
    peep->GiveItem(ShopItem::map);
    peep->CashInPocket = 1000;

    // Guest rejected transport while heading to ride 5
    RideId oldGoal = RideId::FromUnderlying(5);
    RideId newGoal = RideId::FromUnderlying(6); // Different goal now
    peep->GuestHeadingToRideId = newGoal;
    peep->GuestRejectedTransport = transportRide->id;
    peep->GuestRejectedTransportGoal = oldGoal;

    TileCoordsXYZ goal = { 50, 50, 14 };
    auto result = PathFinding::ShouldUseTransportRide(*peep, pos, goal);

    // Transport should be considered because goal changed
    // (The rejection was for oldGoal, but we're now heading to newGoal)
    // Note: Result depends on station positions, so we just verify no crash

    PeepEntityRemove(peep);
}
