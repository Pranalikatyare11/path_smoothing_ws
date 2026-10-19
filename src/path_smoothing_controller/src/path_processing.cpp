#include "path_smoothing_controller/path_processing.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace path_smoothing_controller
{
namespace
{
constexpr double kEpsilon = 1.0e-9;

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

bool finitePoint(const Point2 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

std::vector<Point2> removeConsecutiveDuplicates(const std::vector<Point2> & points)
{
  std::vector<Point2> result;
  result.reserve(points.size());
  for (const auto & point : points) {
    if (!finitePoint(point)) {
      return {};
    }
    const double separation = result.empty() ? 0.0 : distance(result.back(), point);
    if (!std::isfinite(separation)) {
      return {};
    }
    if (result.empty() || separation > kEpsilon) {
      result.push_back(point);
    }
  }
  return result;
}

double angleDifference(double to, double from)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

bool costmapIndexAt(const CostmapGrid & map, const Point2 & point, std::size_t & index)
{
  if (map.width == 0 || map.height == 0 || map.resolution <= 0.0 ||
    map.costs.size() < static_cast<std::size_t>(map.width) * map.height ||
    !finitePoint(point))
  {
    return false;
  }

  const double dx = point.x - map.origin.x;
  const double dy = point.y - map.origin.y;
  const double cosine = std::cos(map.origin.yaw);
  const double sine = std::sin(map.origin.yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }

  const auto cell_x = static_cast<std::uint32_t>(std::floor(local_x / map.resolution));
  const auto cell_y = static_cast<std::uint32_t>(std::floor(local_y / map.resolution));
  if (cell_x >= map.width || cell_y >= map.height) {
    return false;
  }
  index = static_cast<std::size_t>(cell_y) * map.width + cell_x;
  return true;
}

bool costAt(
  const CostmapGrid & map, const Point2 & point, std::uint8_t blocked_threshold)
{
  std::size_t index = 0;
  return costmapIndexAt(map, point, index) && map.costs[index] < blocked_threshold;
}
}  // namespace

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double limitVelocityChange(
  double target_velocity, double previous_velocity, double max_acceleration, double dt)
{
  if (!std::isfinite(target_velocity) || !std::isfinite(previous_velocity) ||
    !std::isfinite(max_acceleration) || max_acceleration < 0.0 ||
    !std::isfinite(dt) || dt < 0.0)
  {
    return 0.0;
  }
  const double max_delta = max_acceleration * dt;
  return std::clamp(target_velocity, previous_velocity - max_delta, previous_velocity + max_delta);
}

GoalAlignmentCommand goalAlignmentCommand(
  const Pose2 & robot_pose, const TrajectoryPoint & goal,
  double position_tolerance, double yaw_tolerance, double max_angular_velocity)
{
  GoalAlignmentCommand command;
  if (!finitePoint({robot_pose.x, robot_pose.y}) || !std::isfinite(robot_pose.yaw) ||
    !std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.heading) ||
    !std::isfinite(position_tolerance) || position_tolerance < 0.0 ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance < 0.0 ||
    !std::isfinite(max_angular_velocity) || max_angular_velocity <= 0.0)
  {
    return command;
  }
  command.position_reached = std::hypot(goal.x - robot_pose.x, goal.y - robot_pose.y) <=
    position_tolerance;
  if (!command.position_reached) {
    return command;
  }
  const double yaw_error = normalizeAngle(goal.heading - robot_pose.yaw);
  command.goal_reached = std::abs(yaw_error) <= yaw_tolerance;
  if (!command.goal_reached) {
    command.angular_velocity = std::clamp(
      1.5 * yaw_error, -max_angular_velocity, max_angular_velocity);
  }
  return command;
}

std::vector<Point2> smoothChaikin(
  const std::vector<Point2> & input, std::size_t iterations, double sample_spacing)
{
  if (!std::isfinite(sample_spacing) || sample_spacing <= 0.0) {
    return {};
  }
  auto points = removeConsecutiveDuplicates(input);
  if (points.size() < 3 || iterations == 0) {
    if (points.size() < 2) {
      return points;
    }
  } else {
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
      std::vector<Point2> cut;
      cut.reserve(points.size() * 2);
      cut.push_back(points.front());
      for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const auto & a = points[i];
        const auto & b = points[i + 1];
        cut.push_back({0.75 * a.x + 0.25 * b.x, 0.75 * a.y + 0.25 * b.y});
        cut.push_back({0.25 * a.x + 0.75 * b.x, 0.25 * a.y + 0.75 * b.y});
      }
      cut.push_back(points.back());
      points = std::move(cut);
    }
  }

  if (points.size() < 2) {
    return points;
  }
  std::vector<Point2> dense;
  dense.push_back(points.front());
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & a = points[i];
    const auto & b = points[i + 1];
    const double segment_length = distance(a, b);
    const std::size_t samples = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(segment_length / sample_spacing)));
    for (std::size_t j = 1; j <= samples; ++j) {
      const double ratio = static_cast<double>(j) / static_cast<double>(samples);
      dense.push_back({a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)});
    }
  }
  dense.front() = input.front();
  dense.back() = input.back();
  return dense;
}

std::vector<Point2> parseWaypointsXY(const std::vector<double> & values)
{
  if (values.size() < 10 || values.size() % 2 != 0) {
    return {};
  }
  std::vector<Point2> points;
  points.reserve(values.size() / 2);
  for (std::size_t i = 0; i < values.size(); i += 2) {
    Point2 point{values[i], values[i + 1]};
    if (!finitePoint(point)) {
      return {};
    }
    points.push_back(point);
  }
  return points;
}

std::vector<Point2> smoothPathSegments(
  const std::vector<std::vector<Point2>> & segments, std::size_t iterations,
  double sample_spacing)
{
  if (segments.empty()) {
    return {};
  }
  std::vector<Point2> result;
  for (const auto & segment : segments) {
    auto smoothed = smoothChaikin(segment, iterations, sample_spacing);
    if (smoothed.empty()) {
      return {};
    }
    if (!result.empty()) {
      if (distance(result.back(), smoothed.front()) > kEpsilon) {
        return {};
      }
      smoothed.erase(smoothed.begin());
    }
    result.insert(result.end(), smoothed.begin(), smoothed.end());
  }
  return result;
}

std::vector<std::vector<Point2>> splitPathAtWaypoints(
  const std::vector<Point2> & path, const std::vector<Point2> & waypoints,
  double match_tolerance)
{
  if (path.empty()) {
    return {};
  }
  if (path.size() < 2) {
    return {};
  }
  if (std::any_of(
      path.begin(), path.end(), [](const auto & point) {
        return !finitePoint(point);
      }) || std::any_of(
      waypoints.begin(), waypoints.end(), [](const auto & point) {
        return !finitePoint(point);
      }))
  {
    return {};
  }
  if (waypoints.empty()) {
    return {path};
  }
  if (!std::isfinite(match_tolerance) || match_tolerance <= 0.0) {
    return {};
  }

  std::vector<std::vector<Point2>> segments;
  std::size_t segment_start = 0;
  Point2 previous_anchor = path.front();
  for (const auto & waypoint : waypoints) {
    // A waypoint already equal to the retained boundary is present. Distinct nearby
    // waypoints must match a later path sample instead of being skipped.
    if (distance(previous_anchor, waypoint) <= kEpsilon) {
      continue;
    }

    if (segment_start + 1 >= path.size()) {
      return {};
    }
    std::size_t closest_index = segment_start + 1;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = segment_start + 1; index < path.size(); ++index) {
      const double candidate_distance = distance(path[index], waypoint);
      if (candidate_distance < closest_distance) {
        closest_distance = candidate_distance;
        closest_index = index;
      }
    }
    if (closest_distance > match_tolerance) {
      return {};
    }

    segments.emplace_back(
      path.begin() + static_cast<std::ptrdiff_t>(segment_start),
      path.begin() + static_cast<std::ptrdiff_t>(closest_index + 1));
    if (segments.size() > 1) {
      segments.back().front() = segments[segments.size() - 2].back();
    }
    // Preserve the configured goal exactly, even if Nav2's endpoint differs slightly.
    segments.back().back() = waypoint;
    segment_start = closest_index;
    previous_anchor = waypoint;
  }

  if (segments.empty()) {
    return {path};
  }
  if (segment_start + 1 < path.size()) {
    segments.emplace_back(
      path.begin() + static_cast<std::ptrdiff_t>(segment_start), path.end());
    segments.back().front() = segments[segments.size() - 2].back();
  }
  return segments;
}

std::size_t advanceWaypointProgress(
  const Pose2 & robot_pose, const std::vector<Point2> & waypoints,
  std::size_t current_index, double reached_tolerance)
{
  current_index = std::min(current_index, waypoints.size());
  if (!finitePoint({robot_pose.x, robot_pose.y}) ||
    !std::isfinite(reached_tolerance) || reached_tolerance <= 0.0)
  {
    return current_index;
  }

  while (current_index < waypoints.size() &&
    distance({robot_pose.x, robot_pose.y}, waypoints[current_index]) <= reached_tolerance)
  {
    ++current_index;
  }
  return current_index;
}

bool pathIsCollisionFree(
  const std::vector<Point2> & path, const CostmapGrid & costmap,
  std::uint8_t blocked_cost_threshold)
{
  if (path.empty() || !std::isfinite(costmap.resolution) || costmap.resolution <= 0.0 ||
    !finitePoint({costmap.origin.x, costmap.origin.y}) ||
    !std::isfinite(costmap.origin.yaw) ||
    costmap.costs.size() < static_cast<std::size_t>(costmap.width) * costmap.height)
  {
    return false;
  }
  if (std::any_of(
      path.begin(), path.end(), [](const auto & point) {
        return !finitePoint(point);
      }))
  {
    return false;
  }
  if (!costAt(costmap, path.front(), blocked_cost_threshold)) {
    return false;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto & a = path[i];
    const auto & b = path[i + 1];
    const double length = distance(a, b);
    const std::size_t samples = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / (costmap.resolution * 0.5))));
    for (std::size_t j = 1; j <= samples; ++j) {
      const double ratio = static_cast<double>(j) / static_cast<double>(samples);
      const Point2 sample{a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)};
      if (!costAt(costmap, sample, blocked_cost_threshold)) {
        return false;
      }
    }
  }
  return true;
}

bool pathIsCollisionFreeInCostmapWindow(
  const std::vector<Point2> & path, const CostmapGrid & costmap,
  std::uint8_t blocked_cost_threshold, double minimum_checked_distance)
{
  if (path.empty() || !std::isfinite(minimum_checked_distance) ||
    minimum_checked_distance < 0.0 ||
    !std::isfinite(costmap.resolution) || costmap.resolution <= 0.0 ||
    !finitePoint({costmap.origin.x, costmap.origin.y}) ||
    !std::isfinite(costmap.origin.yaw) ||
    costmap.costs.size() < static_cast<std::size_t>(costmap.width) * costmap.height ||
    std::any_of(
      path.begin(), path.end(), [](const auto & point) {
        return !finitePoint(point);
      }))
  {
    return false;
  }

  double checked_distance = 0.0;
  std::size_t index = 0;
  if (!costmapIndexAt(costmap, path.front(), index) ||
    costmap.costs[index] >= blocked_cost_threshold)
  {
    return false;
  }

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const auto & start = path[i];
    const auto & end = path[i + 1];
    const double length = distance(start, end);
    const std::size_t samples = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(length / (costmap.resolution * 0.5))));
    const double sample_distance = length / static_cast<double>(samples);
    for (std::size_t j = 1; j <= samples; ++j) {
      const double ratio = static_cast<double>(j) / static_cast<double>(samples);
      const Point2 sample{
        start.x + ratio * (end.x - start.x),
        start.y + ratio * (end.y - start.y)};
      if (checked_distance >= minimum_checked_distance) {
        return true;
      }
      if (!costmapIndexAt(costmap, sample, index)) {
        return checked_distance >= minimum_checked_distance;
      }
      if (costmap.costs[index] >= blocked_cost_threshold) {
        return false;
      }
      checked_distance += sample_distance;
    }
  }
  return true;
}

std::vector<TrajectoryPoint> generateTrajectory(
  const std::vector<Point2> & input, double start_yaw, double goal_yaw,
  const MotionLimits & limits)
{
  if (!std::isfinite(start_yaw) || !std::isfinite(goal_yaw) ||
    !std::isfinite(limits.max_linear_velocity) || limits.max_linear_velocity <= 0.0 ||
    !std::isfinite(limits.max_angular_velocity) || limits.max_angular_velocity <= 0.0 ||
    !std::isfinite(limits.max_linear_acceleration) ||
    limits.max_linear_acceleration <= 0.0 ||
    !std::isfinite(limits.sample_spacing) || limits.sample_spacing <= 0.0)
  {
    return {};
  }
  auto path = removeConsecutiveDuplicates(input);
  if (path.empty()) {
    return {};
  }
  if (path.size() == 1) {
    return {{path.front().x, path.front().y, goal_yaw, 0.0, 0.0, 0.0}};
  }
  if (path.size() == 2) {
    const Point2 start = path.front();
    const Point2 finish = path.back();
    const double length = distance(start, finish);
    const std::size_t segments = std::max<std::size_t>(
      2, static_cast<std::size_t>(std::ceil(length / limits.sample_spacing)));
    path.clear();
    path.reserve(segments + 1);
    for (std::size_t i = 0; i <= segments; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(segments);
      path.push_back(
        {start.x + ratio * (finish.x - start.x),
          start.y + ratio * (finish.y - start.y)});
    }
  }

  const std::size_t count = path.size();
  std::vector<double> arc(count, 0.0);
  std::vector<double> headings(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    arc[i] = arc[i - 1] + distance(path[i - 1], path[i]);
  }
  if (arc.back() <= kEpsilon) {
    return {{path.front().x, path.front().y, goal_yaw, 0.0, 0.0, 0.0}};
  }
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t before = i == 0 ? 0 : i - 1;
    const std::size_t after = i + 1 < count ? i + 1 : count - 1;
    headings[i] = std::atan2(
      path[after].y - path[before].y,
      path[after].x - path[before].x);
  }
  headings.front() = normalizeAngle(start_yaw);
  headings.back() = normalizeAngle(goal_yaw);

  std::vector<double> curvature(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t before = i == 0 ? 0 : i - 1;
    const std::size_t after = i + 1 < count ? i + 1 : count - 1;
    const double delta_s = arc[after] - arc[before];
    if (delta_s > kEpsilon) {
      curvature[i] = angleDifference(headings[after], headings[before]) / delta_s;
    }
  }

  std::vector<double> speeds(count, limits.max_linear_velocity);
  for (std::size_t i = 0; i < count; ++i) {
    const double distance_from_start = arc[i];
    const double distance_to_goal = arc.back() - arc[i];
    speeds[i] = std::min(
      {
        limits.max_linear_velocity,
        std::sqrt(2.0 * limits.max_linear_acceleration * distance_from_start),
        std::sqrt(2.0 * limits.max_linear_acceleration * distance_to_goal)});
    if (std::abs(curvature[i]) > kEpsilon) {
      speeds[i] = std::min(speeds[i], limits.max_angular_velocity / std::abs(curvature[i]));
    }
  }
  speeds.front() = 0.0;
  speeds.back() = 0.0;
  for (std::size_t i = 1; i < count; ++i) {
    const double ds = arc[i] - arc[i - 1];
    speeds[i] = std::min(
      speeds[i], std::sqrt(
        speeds[i - 1] * speeds[i - 1] + 2.0 * limits.max_linear_acceleration * ds));
  }
  for (std::size_t i = count - 1; i-- > 0; ) {
    const double ds = arc[i + 1] - arc[i];
    speeds[i] = std::min(
      speeds[i], std::sqrt(
        speeds[i + 1] * speeds[i + 1] + 2.0 * limits.max_linear_acceleration * ds));
  }

  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(count);
  double time = 0.0;
  trajectory.push_back({path[0].x, path[0].y, headings[0], 0.0, speeds[0], 0.0});
  for (std::size_t i = 1; i < count; ++i) {
    const double ds = arc[i] - arc[i - 1];
    const double average_speed = 0.5 * (speeds[i - 1] + speeds[i]);
    if (average_speed <= kEpsilon) {
      return {};
    }
    time += ds / average_speed;
    trajectory.push_back(
      {path[i].x, path[i].y, headings[i], time,
        speeds[i], curvature[i] * speeds[i]});
  }
  trajectory.back().linear_velocity = 0.0;
  trajectory.back().angular_velocity = 0.0;
  return trajectory;
}

TrackingCommand purePursuit(
  const Pose2 & robot_pose, const std::vector<TrajectoryPoint> & trajectory,
  std::size_t previous_nearest_index, double lookahead_distance,
  double max_linear_velocity, double max_angular_velocity)
{
  TrackingCommand command;
  if (trajectory.empty() || !std::isfinite(robot_pose.x) || !std::isfinite(robot_pose.y) ||
    !std::isfinite(robot_pose.yaw) || !std::isfinite(lookahead_distance) ||
    lookahead_distance <= 0.0 || !std::isfinite(max_linear_velocity) ||
    max_linear_velocity < 0.0 || !std::isfinite(max_angular_velocity) ||
    max_angular_velocity <= 0.0)
  {
    return command;
  }

  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto & point = trajectory[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.heading) || !std::isfinite(point.time_from_start) ||
      point.time_from_start < 0.0 ||
      !std::isfinite(point.linear_velocity) || point.linear_velocity < 0.0 ||
      !std::isfinite(point.angular_velocity) ||
      (i > 0 && point.time_from_start <= trajectory[i - 1].time_from_start))
    {
      return command;
    }
  }

  std::vector<double> arc(trajectory.size(), 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    arc[i] = arc[i - 1] + std::hypot(
      trajectory[i].x - trajectory[i - 1].x,
      trajectory[i].y - trajectory[i - 1].y);
  }
  const std::size_t first = std::min(previous_nearest_index, trajectory.size() - 1);
  std::size_t nearest = first;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = first; i < trajectory.size(); ++i) {
    const double d = std::hypot(trajectory[i].x - robot_pose.x, trajectory[i].y - robot_pose.y);
    if (d < nearest_distance) {
      nearest = i;
      nearest_distance = d;
    }
  }

  const double target_arc = arc[nearest] + lookahead_distance;
  std::size_t target = nearest;
  while (target + 1 < trajectory.size() && arc[target] < target_arc) {
    ++target;
  }
  const double dx = trajectory[target].x - robot_pose.x;
  const double dy = trajectory[target].y - robot_pose.y;
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  const double distance_squared = local_x * local_x + local_y * local_y;
  if (distance_squared <= kEpsilon) {
    return command;
  }

  command.valid = true;
  command.nearest_index = nearest;
  command.cross_track_error = nearest_distance;
  command.tracking_error = nearest_distance;
  if (local_x < 0.0) {
    command.linear_velocity = 0.0;
    command.angular_velocity = std::clamp(
      1.5 * std::atan2(local_y, local_x), -max_angular_velocity, max_angular_velocity);
    return command;
  }
  const std::size_t speed_index = std::min(nearest + 1, trajectory.size() - 1);
  command.linear_velocity = std::clamp(
    trajectory[speed_index].linear_velocity, 0.0, max_linear_velocity);
  if (trajectory.size() > 1 && nearest + 1 >= trajectory.size()) {
    // Keep a low-speed approach available until the node's goal tolerance is met.
    command.linear_velocity = std::max(
      command.linear_velocity, std::min(0.05, max_linear_velocity));
  }
  const double curvature = 2.0 * local_y / distance_squared;
  command.angular_velocity = std::clamp(
    curvature * command.linear_velocity, -max_angular_velocity, max_angular_velocity);
  return command;
}

TrackingCommand timeIndexedTrackingCommand(
  const Pose2 & robot_pose, const std::vector<TrajectoryPoint> & trajectory,
  double elapsed_time, double measured_linear_velocity, double measured_angular_velocity,
  double max_linear_velocity, double max_angular_velocity)
{
  TrackingCommand command;
  if (trajectory.empty() || !finitePoint({robot_pose.x, robot_pose.y}) ||
    !std::isfinite(robot_pose.yaw) || !std::isfinite(elapsed_time) || elapsed_time < 0.0 ||
    !std::isfinite(measured_linear_velocity) || !std::isfinite(measured_angular_velocity) ||
    !std::isfinite(max_linear_velocity) || max_linear_velocity < 0.0 ||
    !std::isfinite(max_angular_velocity) || max_angular_velocity <= 0.0)
  {
    return command;
  }

  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto & point = trajectory[i];
    if (!finitePoint({point.x, point.y}) || !std::isfinite(point.heading) ||
      !std::isfinite(point.time_from_start) || point.time_from_start < 0.0 ||
      !std::isfinite(point.linear_velocity) || point.linear_velocity < 0.0 ||
      !std::isfinite(point.angular_velocity) ||
      (i > 0 && point.time_from_start <= trajectory[i - 1].time_from_start))
    {
      return command;
    }
  }

  TrajectoryPoint reference = trajectory.front();
  std::size_t reference_index = 0;
  if (elapsed_time >= trajectory.back().time_from_start) {
    reference = trajectory.back();
    reference_index = trajectory.size() - 1;
  } else if (elapsed_time > trajectory.front().time_from_start) {
    const auto next = std::upper_bound(
      trajectory.begin(), trajectory.end(), elapsed_time,
      [](double time, const TrajectoryPoint & point) {
        return time < point.time_from_start;
      });
    const auto previous = next - 1;
    const double interval = next->time_from_start - previous->time_from_start;
    const double ratio = (elapsed_time - previous->time_from_start) / interval;
    reference.x = previous->x + ratio * (next->x - previous->x);
    reference.y = previous->y + ratio * (next->y - previous->y);
    reference.heading = normalizeAngle(
      previous->heading + ratio * normalizeAngle(next->heading - previous->heading));
    reference.linear_velocity =
      previous->linear_velocity + ratio * (next->linear_velocity - previous->linear_velocity);
    reference.angular_velocity =
      previous->angular_velocity + ratio * (next->angular_velocity - previous->angular_velocity);
    reference_index = static_cast<std::size_t>(
      std::distance(trajectory.begin(), next));
  }

  const double dx = reference.x - robot_pose.x;
  const double dy = reference.y - robot_pose.y;
  const double cosine = std::cos(robot_pose.yaw);
  const double sine = std::sin(robot_pose.yaw);
  const double longitudinal_error = cosine * dx + sine * dy;
  const double lateral_error = -sine * dx + cosine * dy;
  const double heading_error = normalizeAngle(reference.heading - robot_pose.yaw);

  command.valid = true;
  command.nearest_index = reference_index;
  command.cross_track_error = std::abs(lateral_error);
  command.tracking_error = std::hypot(dx, dy);
  if (std::abs(heading_error) > 0.5 * std::acos(-1.0)) {
    command.angular_velocity = std::clamp(
      2.0 * heading_error +
      0.2 * (reference.angular_velocity - measured_angular_velocity),
      -max_angular_velocity, max_angular_velocity);
    return command;
  }

  command.linear_velocity = std::clamp(
    reference.linear_velocity * std::cos(heading_error) +
    1.5 * longitudinal_error +
    0.5 * (reference.linear_velocity - measured_linear_velocity),
    0.0, max_linear_velocity);
  command.angular_velocity = std::clamp(
    reference.angular_velocity +
    1.5 * lateral_error +
    2.0 * heading_error +
    0.2 * (reference.angular_velocity - measured_angular_velocity),
    -max_angular_velocity, max_angular_velocity);
  return command;
}

}  // namespace path_smoothing_controller
