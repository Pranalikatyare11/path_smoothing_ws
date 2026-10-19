#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "path_smoothing_controller/path_processing.hpp"

namespace path_smoothing_controller
{
namespace
{
CostmapGrid makeFreeMap()
{
  CostmapGrid map;
  map.width = 40;
  map.height = 40;
  map.resolution = 0.1;
  map.origin = {-1.0, -1.0, 0.0};
  map.costs.assign(static_cast<std::size_t>(map.width) * map.height, 0U);
  return map;
}

}  // namespace

TEST(PathSmoothing, PreservesEndpointsAndDensifies)
{
  const std::vector<Point2> input{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const auto smooth = smoothChaikin(input, 2, 0.05);
  ASSERT_GT(smooth.size(), input.size());
  EXPECT_DOUBLE_EQ(smooth.front().x, input.front().x);
  EXPECT_DOUBLE_EQ(smooth.front().y, input.front().y);
  EXPECT_DOUBLE_EQ(smooth.back().x, input.back().x);
  EXPECT_DOUBLE_EQ(smooth.back().y, input.back().y);
  for (std::size_t i = 1; i < smooth.size(); ++i) {
    EXPECT_LE(
      std::hypot(
        smooth[i].x - smooth[i - 1].x,
        smooth[i].y - smooth[i - 1].y), 0.050001);
  }
}

TEST(PathSmoothing, HandlesEmptyDuplicateAndInvalidInputs)
{
  EXPECT_TRUE(smoothChaikin({}, 2, 0.05).empty());
  const auto single = smoothChaikin({{1.0, 2.0}, {1.0, 2.0}}, 3, 0.05);
  ASSERT_EQ(single.size(), 1U);
  EXPECT_DOUBLE_EQ(single.front().x, 1.0);
  EXPECT_TRUE(smoothChaikin({{0.0, 0.0}}, 2, 0.0).empty());
  EXPECT_TRUE(smoothChaikin({{0.0, 0.0}, {NAN, 1.0}}, 1, 0.05).empty());
}

TEST(PathSmoothing, ParsesAndPreservesFiveOrderedWaypointsAcrossLegs)
{
  const auto waypoints = parseWaypointsXY(
    {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0, 2.0, 2.0});
  ASSERT_EQ(waypoints.size(), 5U);
  EXPECT_TRUE(parseWaypointsXY({0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 1.0}).empty());
  EXPECT_TRUE(parseWaypointsXY({0.0, 0.0, 1.0, 0.0, NAN, 1.0, 2.0, 1.0, 2.0, 2.0}).empty());

  std::vector<std::vector<Point2>> legs;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    legs.push_back({waypoints[i - 1], waypoints[i]});
  }
  const auto smooth = smoothPathSegments(legs, 2, 0.05);
  ASSERT_FALSE(smooth.empty());
  std::size_t search_from = 0;
  for (const auto & waypoint : waypoints) {
    bool found_in_order = false;
    for (std::size_t i = search_from; i < smooth.size(); ++i) {
      if (std::hypot(smooth[i].x - waypoint.x, smooth[i].y - waypoint.y) < 1.0e-9) {
        search_from = i + 1;
        found_in_order = true;
        break;
      }
    }
    EXPECT_TRUE(found_in_order);
  }
  EXPECT_TRUE(smoothPathSegments({legs.front(), {{9.0, 9.0}, {10.0, 10.0}}}, 1, 0.05).empty());
}

TEST(PathSmoothing, SplitsAndPreservesCloselySpacedWaypointsInOrder)
{
  std::vector<Point2> planner_path;
  for (std::size_t i = 0; i <= 30; ++i) {
    planner_path.push_back({static_cast<double>(i) * 0.05, 0.0});
  }
  const std::vector<Point2> waypoints{
    {0.25, 0.04}, {0.25, 0.0}, {0.50, 0.04}, {0.75, 0.04}, {1.00, 0.04}};
  constexpr double match_tolerance = 0.10;

  const auto segments = splitPathAtWaypoints(planner_path, waypoints, match_tolerance);
  ASSERT_EQ(segments.size(), waypoints.size() + 1U);
  const auto smoothed = smoothPathSegments(segments, 2, 0.05);
  ASSERT_FALSE(smoothed.empty());

  std::size_t search_from = 0;
  for (const auto & waypoint : waypoints) {
    const auto match = std::find_if(
      smoothed.begin() + static_cast<std::ptrdiff_t>(search_from), smoothed.end(),
      [&waypoint](const auto & point) {
        return std::hypot(point.x - waypoint.x, point.y - waypoint.y) < 1.0e-9;
      });
    ASSERT_NE(match, smoothed.end()) << "missing waypoint " << waypoint.x << ", " << waypoint.y;
    search_from = static_cast<std::size_t>(std::distance(smoothed.begin(), match)) + 1U;
  }

  // These goals are 0.04 m apart and under 0.65 m; both must remain distinct anchors.
  EXPECT_LT(
    std::hypot(
      waypoints[1].x - waypoints[0].x,
      waypoints[1].y - waypoints[0].y), match_tolerance);
  EXPECT_LT(match_tolerance, 0.65);
  EXPECT_TRUE(splitPathAtWaypoints(planner_path, waypoints, 0.01).empty());
  EXPECT_TRUE(splitPathAtWaypoints({{0.0, 0.0}}, waypoints, match_tolerance).empty());
  EXPECT_EQ(splitPathAtWaypoints(planner_path, {}, match_tolerance).size(), 1U);
}

TEST(PathSmoothing, AdvancesOnlyThroughReachedWaypointsInOrder)
{
  const std::vector<Point2> waypoints{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};

  EXPECT_EQ(advanceWaypointProgress({0.05, 0.0, 0.0}, waypoints, 0U, 0.1), 1U);
  EXPECT_EQ(advanceWaypointProgress({2.0, 0.0, 0.0}, waypoints, 1U, 0.1), 1U);
  EXPECT_EQ(advanceWaypointProgress({1.05, 0.0, 0.0}, waypoints, 1U, 0.1), 2U);
  EXPECT_EQ(advanceWaypointProgress({2.0, 0.0, 0.0}, waypoints, 2U, 0.1), 3U);
  EXPECT_EQ(advanceWaypointProgress({4.0, 0.0, 0.0}, waypoints, 4U, 0.1), 5U);
  EXPECT_EQ(advanceWaypointProgress({0.0, 0.0, 0.0}, waypoints, 6U, 0.1), 5U);
  EXPECT_EQ(advanceWaypointProgress({NAN, 0.0, 0.0}, waypoints, 1U, 0.1), 1U);
}

TEST(CollisionChecking, RejectsBlockedCellsAndOutOfBoundsPoints)
{
  auto map = makeFreeMap();
  EXPECT_TRUE(pathIsCollisionFree({{0.0, 0.0}, {1.0, 0.0}}, map));

  EXPECT_FALSE(pathIsCollisionFree({{0.0, 0.0}, {NAN, 0.0}}, map));
  auto invalid_map = map;
  invalid_map.resolution = INFINITY;
  EXPECT_FALSE(pathIsCollisionFree({{0.0, 0.0}}, invalid_map));

  const std::size_t obstacle = static_cast<std::size_t>(map.width) * 15U + 15U;
  map.costs[obstacle] = 255U;
  EXPECT_FALSE(pathIsCollisionFree({{0.0, 0.5}, {1.0, 0.5}}, map));
  EXPECT_FALSE(pathIsCollisionFree({{0.0, 0.0}, {4.0, 0.0}}, map));
}

TEST(CollisionChecking, ValidatesRollingWindowPrefixAndRejectsInsufficientCoverage)
{
  auto map = makeFreeMap();
  const std::vector<Point2> long_path{{0.0, 0.0}, {1.5, 0.0}};
  EXPECT_TRUE(pathIsCollisionFreeInCostmapWindow(long_path, map, 200U, 0.75));
  EXPECT_FALSE(
    pathIsCollisionFreeInCostmapWindow(
      {{2.5, 0.0}, {3.5, 0.0}}, map, 200U, 0.75));

  const std::size_t obstacle = static_cast<std::size_t>(map.width) * 10U + 12U;
  map.costs[obstacle] = 220U;
  EXPECT_FALSE(pathIsCollisionFreeInCostmapWindow(long_path, map, 200U, 0.75));
}

TEST(CollisionChecking, RollingWindowCheckIgnoresObstaclesBeyondCheckedPrefix)
{
  auto map = makeFreeMap();
  const std::size_t obstacle = static_cast<std::size_t>(map.width) * 10U + 25U;
  map.costs[obstacle] = 220U;

  EXPECT_TRUE(
    pathIsCollisionFreeInCostmapWindow(
      {{0.0, 0.0}, {1.5, 0.0}}, map, 200U, 0.75));
}

TEST(CollisionChecking, AllowsInflationCostsBelowInscribedObstacleThreshold)
{
  auto map = makeFreeMap();
  const std::size_t inflated_cell = static_cast<std::size_t>(map.width) * 10U + 15U;
  map.costs[inflated_cell] = 220U;

  EXPECT_TRUE(pathIsCollisionFree({{0.0, 0.0}, {1.0, 0.0}}, map, 253U));
  EXPECT_FALSE(pathIsCollisionFree({{0.0, 0.0}, {1.0, 0.0}}, map, 200U));
}

TEST(CollisionChecking, ReplannedPathCanAvoidANewObstacleAndRegenerateTrajectory)
{
  auto map = makeFreeMap();
  const std::size_t obstacle = static_cast<std::size_t>(map.width) * 10U + 20U;
  map.costs[obstacle] = 220U;

  const std::vector<Point2> blocked_path{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<Point2> alternate_path{
    {0.0, 0.0}, {0.7, 0.6}, {1.3, 0.6}, {2.0, 0.0}};
  EXPECT_TRUE(pathIsCollisionFree(blocked_path, map));
  EXPECT_FALSE(pathIsCollisionFree(blocked_path, map, 200U));
  ASSERT_TRUE(pathIsCollisionFree(alternate_path, map, 200U));

  const auto smoothed = smoothChaikin(alternate_path, 2, 0.05);
  ASSERT_FALSE(smoothed.empty());
  ASSERT_TRUE(pathIsCollisionFree(smoothed, map, 200U));
  const auto trajectory = generateTrajectory(smoothed, 0.0, 0.0, MotionLimits{});
  ASSERT_FALSE(trajectory.empty());

  std::vector<Point2> trajectory_path;
  trajectory_path.reserve(trajectory.size());
  for (const auto & point : trajectory) {
    trajectory_path.push_back({point.x, point.y});
  }
  EXPECT_TRUE(pathIsCollisionFree(trajectory_path, map, 200U));
  EXPECT_DOUBLE_EQ(trajectory.front().time_from_start, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.back().linear_velocity, 0.0);
}

TEST(Trajectory, TimestampsIncreaseAndSpeedsRespectLimits)
{
  MotionLimits limits;
  limits.max_linear_velocity = 0.20;
  limits.max_angular_velocity = 0.75;
  limits.max_linear_acceleration = 0.4;
  limits.sample_spacing = 0.05;
  const auto trajectory = generateTrajectory(
    {{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, limits);
  ASSERT_GT(trajectory.size(), 2U);
  EXPECT_DOUBLE_EQ(trajectory.front().time_from_start, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.back().linear_velocity, 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GT(trajectory[i].time_from_start, trajectory[i - 1].time_from_start);
  }
  for (const auto & point : trajectory) {
    EXPECT_GE(point.linear_velocity, 0.0);
    EXPECT_LE(point.linear_velocity, limits.max_linear_velocity + 1.0e-9);
    EXPECT_LE(std::abs(point.angular_velocity), limits.max_angular_velocity + 1.0e-9);
    EXPECT_TRUE(std::isfinite(point.heading));
  }
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double ds = std::hypot(
      trajectory[i].x - trajectory[i - 1].x,
      trajectory[i].y - trajectory[i - 1].y);
    const double speed_squared_delta = std::abs(
      trajectory[i].linear_velocity * trajectory[i].linear_velocity -
      trajectory[i - 1].linear_velocity * trajectory[i - 1].linear_velocity);
    EXPECT_LE(
      speed_squared_delta,
      2.0 * limits.max_linear_acceleration * ds + 1.0e-9);
    const double dt = trajectory[i].time_from_start - trajectory[i - 1].time_from_start;
    EXPECT_LE(
      std::abs(trajectory[i].linear_velocity - trajectory[i - 1].linear_velocity) / dt,
      limits.max_linear_acceleration + 1.0e-9);
  }
}

TEST(Trajectory, HeadingProfileUsesPathTangentAndRequestedEndpointYaw)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  const auto trajectory = generateTrajectory(
    {{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}}, half_pi, -half_pi, MotionLimits{});
  ASSERT_GE(trajectory.size(), 3U);
  EXPECT_NEAR(trajectory.front().heading, half_pi, 1.0e-9);
  EXPECT_NEAR(trajectory.back().heading, -half_pi, 1.0e-9);
  EXPECT_NEAR(trajectory[trajectory.size() / 2].heading, half_pi / 2.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(trajectory.back().linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.back().angular_velocity, 0.0);
  for (const auto & point : trajectory) {
    EXPECT_TRUE(std::isfinite(point.angular_velocity));
    EXPECT_LE(std::abs(point.angular_velocity), 1.0 + 1.0e-9);
  }
}

TEST(Trajectory, HandlesSingletonAndRejectsInvalidLimits)
{
  MotionLimits limits;
  const auto stationary = generateTrajectory({{2.0, 3.0}}, 0.0, 1.0, limits);
  ASSERT_EQ(stationary.size(), 1U);
  EXPECT_DOUBLE_EQ(stationary.front().linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(stationary.front().angular_velocity, 0.0);

  limits.max_linear_acceleration = 0.0;
  EXPECT_TRUE(generateTrajectory({{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, limits).empty());

  limits.max_linear_acceleration = 0.5;
  const auto duplicate_segment = generateTrajectory(
    {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, limits);
  ASSERT_GT(duplicate_segment.size(), 2U);
  EXPECT_DOUBLE_EQ(duplicate_segment.front().x, 0.0);
  EXPECT_DOUBLE_EQ(duplicate_segment.back().x, 1.0);
}

TEST(Trajectory, LimitsVelocityAccelerationAndRejectsInvalidInputs)
{
  EXPECT_NEAR(limitVelocityChange(1.0, 0.0, 2.0, 0.1), 0.2, 1.0e-9);
  EXPECT_NEAR(limitVelocityChange(-1.0, 0.3, 2.0, 0.1), 0.1, 1.0e-9);
  EXPECT_DOUBLE_EQ(limitVelocityChange(1.0, -0.2, 2.0, 0.0), -0.2);
  EXPECT_DOUBLE_EQ(limitVelocityChange(NAN, 0.0, 2.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(limitVelocityChange(1.0, 0.0, -1.0, 0.1), 0.0);
}

TEST(GoalTracking, StopsAtPositionAndAlignsBeforeReportingCompletion)
{
  const TrajectoryPoint goal{1.0, 2.0, 0.0, 5.0, 0.0, 0.0};
  const auto aligned = goalAlignmentCommand({1.05, 2.0, 0.05}, goal, 0.1, 0.1, 0.8);
  EXPECT_TRUE(aligned.position_reached);
  EXPECT_TRUE(aligned.goal_reached);
  EXPECT_DOUBLE_EQ(aligned.angular_velocity, 0.0);

  const auto rotate = goalAlignmentCommand(
    {1.05, 2.0, 1.0}, goal, 0.1, 0.1, 0.8);
  EXPECT_TRUE(rotate.position_reached);
  EXPECT_FALSE(rotate.goal_reached);
  EXPECT_LE(std::abs(rotate.angular_velocity), 0.8);
  EXPECT_NE(rotate.angular_velocity, 0.0);

  const auto approach = goalAlignmentCommand({0.0, 2.0, 0.0}, goal, 0.1, 0.1, 0.8);
  EXPECT_FALSE(approach.position_reached);
  EXPECT_FALSE(approach.goal_reached);
  EXPECT_DOUBLE_EQ(approach.angular_velocity, 0.0);
}

TEST(PurePursuit, ProducesBoundedForwardCommand)
{
  const std::vector<TrajectoryPoint> trajectory{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.0, 0.0, 2.0, 0.3, 0.0},
    {1.0, 0.0, 0.0, 4.0, 0.0, 0.0}};
  const auto command = purePursuit({0.0, 0.0, 0.0}, trajectory, 0, 0.3, 0.2, 0.8);
  ASSERT_TRUE(command.valid);
  EXPECT_GT(command.linear_velocity, 0.0);
  EXPECT_LE(command.linear_velocity, 0.2);
  EXPECT_LE(std::abs(command.angular_velocity), 0.8);
  EXPECT_NEAR(command.cross_track_error, 0.0, 1.0e-9);
}

TEST(PurePursuit, InvalidInputStopsAndBehindTargetRotatesInPlace)
{
  EXPECT_FALSE(purePursuit({0.0, 0.0, 0.0}, {}, 0, 0.3, 0.2, 0.8).valid);
  const std::vector<TrajectoryPoint> trajectory{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 2.0, 0.2, 0.0}};
  const auto command = purePursuit(
    {0.0, 0.0, 3.141592653589793},
    trajectory, 0, 0.3, 0.2, 0.8);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.linear_velocity, 0.0);
  EXPECT_LE(std::abs(command.angular_velocity), 0.8);

  auto invalid_time = trajectory;
  invalid_time.back().time_from_start = invalid_time.front().time_from_start;
  EXPECT_FALSE(
    purePursuit(
      {0.0, 0.0, 0.0}, invalid_time, 0, 0.3, 0.2, 0.8).valid);

  const auto zero_speed_limit = purePursuit(
    {0.0, 0.0, 0.0}, trajectory, 0, 0.3, 0.0, 0.8);
  ASSERT_TRUE(zero_speed_limit.valid);
  EXPECT_DOUBLE_EQ(zero_speed_limit.linear_velocity, 0.0);
}

TEST(PurePursuit, ContinuesMovingWhenLookaheadReachesZeroSpeedGoalSample)
{
  MotionLimits limits;
  limits.sample_spacing = 0.1;
  limits.max_linear_velocity = 0.2;
  limits.max_linear_acceleration = 0.1;
  const auto trajectory = generateTrajectory(
    {{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, limits);
  ASSERT_GT(trajectory.size(), 3U);
  const auto command = purePursuit(
    {0.8, 0.0, 0.0}, trajectory, 0, 0.45, 0.2, 1.0);
  ASSERT_TRUE(command.valid);
  EXPECT_GT(command.linear_velocity, 0.0);
  EXPECT_LT(command.linear_velocity, limits.max_linear_velocity);
}

TEST(PurePursuit, CreepsTowardEndpointUntilGoalToleranceIsMet)
{
  MotionLimits limits;
  limits.sample_spacing = 0.1;
  const auto trajectory = generateTrajectory(
    {{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, limits);
  ASSERT_GT(trajectory.size(), 3U);
  const auto command = purePursuit(
    {0.98, 0.22, 0.0}, trajectory, 0, 0.45, 0.26, 1.0);
  ASSERT_TRUE(command.valid);
  EXPECT_GT(command.linear_velocity, 0.0);
  EXPECT_LE(command.linear_velocity, 0.05);
}

TEST(TimeIndexedTracking, SelectsInterpolatedReferencesByElapsedTime)
{
  const std::vector<TrajectoryPoint> trajectory{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 1.0, 0.2, 0.6},
    {2.0, 0.0, 0.0, 2.0, 0.0, 0.2}};

  const auto early = timeIndexedTrackingCommand(
    {0.0, 0.0, 0.0}, trajectory, 0.5, 0.0, 0.0, 10.0, 1.0);
  const auto late = timeIndexedTrackingCommand(
    {0.0, 0.0, 0.0}, trajectory, 1.5, 0.0, 0.0, 10.0, 1.0);

  ASSERT_TRUE(early.valid);
  ASSERT_TRUE(late.valid);
  EXPECT_EQ(early.nearest_index, 1U);
  EXPECT_EQ(late.nearest_index, 2U);
  EXPECT_NEAR(early.tracking_error, 0.5, 1.0e-9);
  EXPECT_NEAR(late.tracking_error, 1.5, 1.0e-9);
  EXPECT_GT(late.linear_velocity, early.linear_velocity);
  EXPECT_GT(late.angular_velocity, early.angular_velocity);
}

TEST(TimeIndexedTracking, UsesOdometryFeedbackAndEnforcesVelocityLimits)
{
  const std::vector<TrajectoryPoint> trajectory{
    {0.0, 0.0, 0.0, 0.0, 0.2, 0.3},
    {1.0, 0.0, 0.0, 1.0, 0.2, 0.3}};

  const auto matched = timeIndexedTrackingCommand(
    {0.5, 0.0, 0.0}, trajectory, 0.5, 0.2, 0.3, 0.5, 0.4);
  const auto lagging = timeIndexedTrackingCommand(
    {0.5, 0.2, 0.0}, trajectory, 0.5, 0.0, 0.0, 0.5, 0.4);
  const auto angular_feedback = timeIndexedTrackingCommand(
    {0.5, 0.0, 0.0}, trajectory, 0.5, 0.2, 0.0, 0.5, 0.4);
  const auto limited = timeIndexedTrackingCommand(
    {0.5, 0.2, 0.0}, trajectory, 0.5, 0.0, 0.0, 0.12, 0.25);

  ASSERT_TRUE(matched.valid);
  ASSERT_TRUE(lagging.valid);
  ASSERT_TRUE(angular_feedback.valid);
  ASSERT_TRUE(limited.valid);
  EXPECT_NEAR(matched.linear_velocity, 0.2, 1.0e-9);
  EXPECT_NEAR(matched.angular_velocity, 0.3, 1.0e-9);
  EXPECT_GT(lagging.linear_velocity, matched.linear_velocity);
  EXPECT_GT(angular_feedback.angular_velocity, matched.angular_velocity);
  EXPECT_LE(limited.linear_velocity, 0.12);
  EXPECT_LE(std::abs(limited.angular_velocity), 0.25);
  EXPECT_NEAR(lagging.cross_track_error, 0.2, 1.0e-9);
}

TEST(TimeIndexedTracking, HoldsTheFinalReferenceAndRejectsInvalidTiming)
{
  const std::vector<TrajectoryPoint> trajectory{
    {0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
    {1.0, 0.0, 0.0, 1.0, 0.0, 0.0}};

  const auto final = timeIndexedTrackingCommand(
    {0.9, 0.0, 0.0}, trajectory, 4.0, 0.0, 0.0, 0.5, 1.0);
  ASSERT_TRUE(final.valid);
  EXPECT_EQ(final.nearest_index, 1U);
  EXPECT_GT(final.linear_velocity, 0.0);
  EXPECT_FALSE(
    timeIndexedTrackingCommand(
      {0.0, 0.0, 0.0}, trajectory, -0.1, 0.0, 0.0, 0.5, 1.0).valid);

  auto invalid = trajectory;
  invalid.back().time_from_start = invalid.front().time_from_start;
  EXPECT_FALSE(
    timeIndexedTrackingCommand(
      {0.0, 0.0, 0.0}, invalid, 0.5, 0.0, 0.0, 0.5, 1.0).valid);
}

TEST(TimeIndexedTracking, ClosedLoopTracksGeneratedProfileAndStopsAtGoal)
{
  MotionLimits limits;
  limits.max_linear_velocity = 0.2;
  limits.max_angular_velocity = 0.8;
  limits.max_linear_acceleration = 0.4;
  limits.sample_spacing = 0.04;
  const auto smoothed_path = smoothChaikin(
    {{0.0, 0.0}, {0.5, 0.0}, {0.8, 0.3}}, 2, limits.sample_spacing);
  const auto trajectory = generateTrajectory(
    smoothed_path, 0.0, std::atan2(0.3, 0.3), limits);
  ASSERT_GT(trajectory.size(), 2U);

  Pose2 robot_pose{0.0, 0.0, 0.0};
  double measured_linear_velocity = 0.0;
  double measured_angular_velocity = 0.0;
  double elapsed_time = 0.0;
  bool goal_reached = false;
  constexpr double step = 0.05;
  for (std::size_t iteration = 0; iteration < 1000; ++iteration) {
    const auto goal = goalAlignmentCommand(
      robot_pose, trajectory.back(), 0.04, 0.08, limits.max_angular_velocity);
    double linear_velocity = 0.0;
    double angular_velocity = goal.angular_velocity;
    if (!goal.position_reached) {
      const auto command = timeIndexedTrackingCommand(
        robot_pose, trajectory, elapsed_time,
        measured_linear_velocity, measured_angular_velocity,
        limits.max_linear_velocity, limits.max_angular_velocity);
      ASSERT_TRUE(command.valid);
      linear_velocity = command.linear_velocity;
      angular_velocity = command.angular_velocity;
    }
    linear_velocity = std::clamp(
      linear_velocity, measured_linear_velocity - limits.max_linear_acceleration * step,
      measured_linear_velocity + limits.max_linear_acceleration * step);
    linear_velocity = std::clamp(linear_velocity, 0.0, limits.max_linear_velocity);
    angular_velocity = std::clamp(
      angular_velocity, -limits.max_angular_velocity, limits.max_angular_velocity);

    robot_pose.x += linear_velocity * std::cos(robot_pose.yaw) * step;
    robot_pose.y += linear_velocity * std::sin(robot_pose.yaw) * step;
    robot_pose.yaw = normalizeAngle(robot_pose.yaw + angular_velocity * step);
    measured_linear_velocity = linear_velocity;
    measured_angular_velocity = angular_velocity;
    elapsed_time += step;

    if (goal.goal_reached && measured_linear_velocity < 0.01 &&
      std::abs(measured_angular_velocity) < 0.01)
    {
      goal_reached = true;
      break;
    }
  }

  EXPECT_TRUE(goal_reached);
  EXPECT_LE(
    std::hypot(
      trajectory.back().x - robot_pose.x, trajectory.back().y - robot_pose.y), 0.04);
  EXPECT_LE(elapsed_time, trajectory.back().time_from_start + 4.0);
  EXPECT_LE(measured_linear_velocity, limits.max_linear_velocity);
}

}  // namespace path_smoothing_controller
