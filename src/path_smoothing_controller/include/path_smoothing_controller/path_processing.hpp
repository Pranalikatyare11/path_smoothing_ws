#ifndef TESTBED_NAVIGATION__PATH_PROCESSING_HPP_
#define TESTBED_NAVIGATION__PATH_PROCESSING_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace path_smoothing_controller
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CostmapGrid
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.0};
  Pose2 origin;
  std::vector<std::uint8_t> costs;
};

struct MotionLimits
{
  double max_linear_velocity{0.26};
  double max_angular_velocity{1.0};
  double max_linear_acceleration{0.5};
  double sample_spacing{0.05};
};

struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double time_from_start{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

struct TrackingCommand
{
  bool valid{false};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  std::size_t nearest_index{0};
  double cross_track_error{0.0};
  double tracking_error{0.0};
};

struct GoalAlignmentCommand
{
  bool position_reached{false};
  bool goal_reached{false};
  double angular_velocity{0.0};
};

// Endpoint-preserving Chaikin corner cutting followed by arc-length resampling.
std::vector<Point2> smoothChaikin(
  const std::vector<Point2> & waypoints, std::size_t iterations, double sample_spacing);

// Parse [x0, y0, x1, y1, ...]; requires at least five finite ordered points.
std::vector<Point2> parseWaypointsXY(const std::vector<double> & values);

// Smooth each planned leg independently so every shared waypoint is preserved.
std::vector<Point2> smoothPathSegments(
  const std::vector<std::vector<Point2>> & segments, std::size_t iterations,
  double sample_spacing);

// Split a planner path at every configured waypoint in order. Each segment endpoint is
// snapped to its waypoint when it is within match_tolerance; returns empty on a mismatch.
std::vector<std::vector<Point2>> splitPathAtWaypoints(
  const std::vector<Point2> & path, const std::vector<Point2> & waypoints,
  double match_tolerance);

std::size_t advanceWaypointProgress(
  const Pose2 & robot_pose, const std::vector<Point2> & waypoints,
  std::size_t current_index, double reached_tolerance);

// Treats unknown (255), inscribed (253), and lethal (254) cells as blocked by default.
bool pathIsCollisionFree(
  const std::vector<Point2> & path, const CostmapGrid & costmap,
  std::uint8_t blocked_cost_threshold = 253);

// Checks the path prefix inside a rolling costmap window. The path must remain inside and
// collision-free through minimum_checked_distance, or be collision-free through its endpoint
// when the endpoint is nearer.
bool pathIsCollisionFreeInCostmapWindow(
  const std::vector<Point2> & path, const CostmapGrid & costmap,
  std::uint8_t blocked_cost_threshold, double minimum_checked_distance);

// Builds a trapezoidal/triangular speed profile with zero speed at both ends.
std::vector<TrajectoryPoint> generateTrajectory(
  const std::vector<Point2> & path, double start_yaw, double goal_yaw,
  const MotionLimits & limits);

// Spatial Pure Pursuit over trajectory samples. Selects references by path progress and
// applies the generated speed profile; it does not select samples by elapsed time.
TrackingCommand purePursuit(
  const Pose2 & robot_pose, const std::vector<TrajectoryPoint> & trajectory,
  std::size_t previous_nearest_index, double lookahead_distance,
  double max_linear_velocity, double max_angular_velocity);

// Interpolate the reference by elapsed trajectory time and apply pose/odometry feedback.
TrackingCommand timeIndexedTrackingCommand(
  const Pose2 & robot_pose, const std::vector<TrajectoryPoint> & trajectory,
  double elapsed_time, double measured_linear_velocity, double measured_angular_velocity,
  double max_linear_velocity, double max_angular_velocity);

GoalAlignmentCommand goalAlignmentCommand(
  const Pose2 & robot_pose, const TrajectoryPoint & goal,
  double position_tolerance, double yaw_tolerance, double max_angular_velocity);

double normalizeAngle(double angle);

double limitVelocityChange(
  double target_velocity, double previous_velocity, double max_acceleration, double dt);

}  // namespace path_smoothing_controller

#endif  // TESTBED_NAVIGATION__PATH_PROCESSING_HPP_
