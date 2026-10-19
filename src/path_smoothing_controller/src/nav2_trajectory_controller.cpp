#include "path_smoothing_controller/nav2_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace path_smoothing_controller
{
namespace
{
geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

void Nav2TrajectoryController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("Could not lock controller server lifecycle node");
  }
  if (!costmap_ros) {
    throw nav2_core::PlannerException("Controller Server supplied a null costmap");
  }
  parent_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  local_costmap_ = costmap_ros_->getCostmap();

  auto declare_parameter = [this](const std::string & key, const auto & value) {
      const auto parameter_name = plugin_name_ + "." + key;
      if (!node_->has_parameter(parameter_name)) {
        node_->declare_parameter(parameter_name, value);
      }
    };
  declare_parameter("max_linear_velocity", 0.26);
  declare_parameter("max_angular_velocity", 1.0);
  declare_parameter("max_linear_acceleration", 0.5);
  declare_parameter("sample_spacing", 0.05);
  declare_parameter("lookahead_distance", 0.45);
  declare_parameter("waypoint_match_tolerance", 0.10);
  declare_parameter("smoothing_iterations", 2);
  declare_parameter("blocked_cost_threshold", 253);
  declare_parameter("dynamic_obstacle_cost_threshold", 253);
  declare_parameter("goal_tolerance", 0.20);
  declare_parameter("yaw_tolerance", 0.20);
  declare_parameter("replan_retry_interval", 1.0);
  declare_parameter(
    "planner_through_poses_action", std::string("/compute_path_through_poses"));
  declare_parameter("planner_id", std::string("GridBased"));
  declare_parameter(
    "waypoints_xy", std::vector<double>{
      0.8, 5.0, 1.4, 5.3, 2.0, 5.0, 2.0, 4.4, 1.4, 4.2});

  node_->get_parameter(plugin_name_ + ".max_linear_velocity", base_max_linear_velocity_);
  current_max_linear_velocity_ = base_max_linear_velocity_;
  node_->get_parameter(plugin_name_ + ".max_angular_velocity", limits_.max_angular_velocity);
  node_->get_parameter(
    plugin_name_ + ".max_linear_acceleration", limits_.max_linear_acceleration);
  node_->get_parameter(plugin_name_ + ".sample_spacing", limits_.sample_spacing);
  node_->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node_->get_parameter(plugin_name_ + ".waypoint_match_tolerance", waypoint_match_tolerance_);
  if (!std::isfinite(waypoint_match_tolerance_) || waypoint_match_tolerance_ <= 0.0) {
    throw nav2_core::PlannerException("waypoint_match_tolerance must be finite and positive");
  }
  node_->get_parameter(plugin_name_ + ".smoothing_iterations", smoothing_iterations_);
  int blocked_threshold = 253;
  node_->get_parameter(plugin_name_ + ".blocked_cost_threshold", blocked_threshold);
  blocked_cost_threshold_ = static_cast<std::uint8_t>(
    std::clamp(blocked_threshold, 0, 255));
  int dynamic_obstacle_threshold = 253;
  node_->get_parameter(
    plugin_name_ + ".dynamic_obstacle_cost_threshold", dynamic_obstacle_threshold);
  dynamic_obstacle_cost_threshold_ = static_cast<std::uint8_t>(
    std::clamp(dynamic_obstacle_threshold, 1, 255));
  node_->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  node_->get_parameter(plugin_name_ + ".yaw_tolerance", yaw_tolerance_);
  node_->get_parameter(plugin_name_ + ".replan_retry_interval", replan_retry_interval_);
  node_->get_parameter(
    plugin_name_ + ".planner_through_poses_action", planner_through_poses_action_);
  node_->get_parameter(plugin_name_ + ".planner_id", planner_id_);
  if (!std::isfinite(base_max_linear_velocity_) || base_max_linear_velocity_ <= 0.0 ||
    !std::isfinite(limits_.max_angular_velocity) || limits_.max_angular_velocity <= 0.0 ||
    !std::isfinite(limits_.max_linear_acceleration) ||
    limits_.max_linear_acceleration <= 0.0 ||
    !std::isfinite(limits_.sample_spacing) || limits_.sample_spacing <= 0.0 ||
    !std::isfinite(lookahead_distance_) || lookahead_distance_ <= 0.0 ||
    !std::isfinite(goal_tolerance_) || goal_tolerance_ < 0.0 ||
    !std::isfinite(yaw_tolerance_) || yaw_tolerance_ < 0.0 ||
    !std::isfinite(replan_retry_interval_) || replan_retry_interval_ <= 0.0 ||
    smoothing_iterations_ < 0)
  {
    throw nav2_core::PlannerException("Controller motion and tolerance parameters are invalid");
  }
  std::string global_costmap_topic;
  declare_parameter(
    "global_costmap_topic", std::string("/global_costmap/costmap_raw"));
  node_->get_parameter(plugin_name_ + ".global_costmap_topic", global_costmap_topic);
  std::vector<double> waypoint_values;
  node_->get_parameter(plugin_name_ + ".waypoints_xy", waypoint_values);
  configured_waypoints_ = parseWaypointsXY(waypoint_values);
  if (configured_waypoints_.empty()) {
    throw nav2_core::PlannerException(
            "waypoints_xy must contain at least five finite ordered (x, y) waypoint pairs");
  }

  costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    global_costmap_topic, rclcpp::QoS(1),
    std::bind(&Nav2TrajectoryController::onGlobalCostmap, this, std::placeholders::_1));
  planner_client_ = rclcpp_action::create_client<ComputePath>(
    node_, planner_through_poses_action_);
  planned_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/nav2_planned_path", rclcpp::QoS(1).transient_local());
  smoothed_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/smoothed_path", rclcpp::QoS(1).transient_local());
  trajectory_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/trajectory_path", rclcpp::QoS(1).transient_local());
  trajectory_pub_ = node_->create_publisher<msg::Trajectory>(
    "/trajectory", rclcpp::QoS(1).transient_local());
  actual_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/actual_path", rclcpp::QoS(1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured time-indexed trajectory controller '%s' "
    "(v=%.2f, w=%.2f, a=%.2f, waypoints=%zu)",
    plugin_name_.c_str(), base_max_linear_velocity_, limits_.max_angular_velocity,
    limits_.max_linear_acceleration, configured_waypoints_.size());
}

void Nav2TrajectoryController::cleanup()
{
  planned_path_pub_.reset();
  smoothed_path_pub_.reset();
  trajectory_path_pub_.reset();
  trajectory_pub_.reset();
  actual_path_pub_.reset();
  costmap_subscription_.reset();
  planner_client_.reset();
  costmap_ros_.reset();
  local_costmap_ = nullptr;
  node_.reset();
  have_plan_ = false;
  trajectory_clock_started_ = false;
  resetReplanState();
  original_path_.clear();
  smoothed_path_.clear();
  trajectory_.clear();
}

void Nav2TrajectoryController::activate()
{
  planned_path_pub_->on_activate();
  smoothed_path_pub_->on_activate();
  trajectory_path_pub_->on_activate();
  trajectory_pub_->on_activate();
  actual_path_pub_->on_activate();
}

void Nav2TrajectoryController::deactivate()
{
  planned_path_pub_->on_deactivate();
  smoothed_path_pub_->on_deactivate();
  trajectory_path_pub_->on_deactivate();
  trajectory_pub_->on_deactivate();
  actual_path_pub_->on_deactivate();
}

void Nav2TrajectoryController::onGlobalCostmap(
  const nav2_msgs::msg::Costmap::SharedPtr message)
{
  if (message->header.frame_id.empty() || message->metadata.size_x == 0 ||
    message->metadata.size_y == 0 || message->metadata.resolution <= 0.0 ||
    message->data.size() <
    static_cast<std::size_t>(message->metadata.size_x) * message->metadata.size_y)
  {
    return;
  }

  CostmapGrid updated;
  updated.width = message->metadata.size_x;
  updated.height = message->metadata.size_y;
  updated.resolution = message->metadata.resolution;
  updated.origin.x = message->metadata.origin.position.x;
  updated.origin.y = message->metadata.origin.position.y;
  updated.origin.yaw = tf2::getYaw(message->metadata.origin.orientation);
  updated.costs = message->data;
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  global_costmap_ = std::move(updated);
  global_costmap_frame_ = message->header.frame_id;
  global_costmap_stamp_ = rclcpp::Time(message->header.stamp, node_->get_clock()->get_clock_type());
  have_global_costmap_ = true;
}

bool Nav2TrajectoryController::snapshotGlobalCostmap(
  CostmapGrid & output, std::string & frame)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!have_global_costmap_ || (node_->now() - global_costmap_stamp_).seconds() > 2.0) {
    return false;
  }
  output = global_costmap_;
  frame = global_costmap_frame_;
  return true;
}

bool Nav2TrajectoryController::buildTrajectoryFromPath(
  const nav_msgs::msg::Path & path, const CostmapGrid & costmap,
  const std::string & costmap_frame, const std::vector<Point2> & required_waypoints,
  std::uint8_t collision_threshold,
  std::vector<Point2> & original, std::vector<Point2> & smoothed,
  std::vector<TrajectoryPoint> & trajectory, std::string & error) const
{
  if (path.poses.size() < 2 || path.header.frame_id.empty()) {
    error = "planner returned a path with fewer than two poses or no frame";
    return false;
  }
  if (path.header.frame_id != costmap_frame) {
    error = "planner path frame does not match the global costmap frame";
    return false;
  }

  original.clear();
  original.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const auto & point = pose.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      error = "planner path contains a non-finite pose";
      return false;
    }
    original.push_back({point.x, point.y});
  }
  if (!pathIsCollisionFree(original, costmap, collision_threshold)) {
    error = "planner path intersects blocked or unknown costmap cells";
    return false;
  }

  std::vector<std::vector<Point2>> segments;
  if (!required_waypoints.empty()) {
    segments = splitPathAtWaypoints(
      original, required_waypoints, waypoint_match_tolerance_);
    if (segments.empty()) {
      error = "planner path does not match every remaining waypoint in order";
      return false;
    }
  } else {
    segments.push_back(original);
  }

  smoothed = smoothPathSegments(
    segments, static_cast<std::size_t>(std::max(0, smoothing_iterations_)),
    limits_.sample_spacing);
  if (smoothed.empty()) {
    error = "path smoothing failed";
    return false;
  }
  if (!pathIsCollisionFree(smoothed, costmap, collision_threshold)) {
    RCLCPP_WARN(
      node_->get_logger(), "Smoothed path intersects a blocked cell; retrying without smoothing");
    smoothed = smoothPathSegments(segments, 0, limits_.sample_spacing);
  }
  if (!pathIsCollisionFree(smoothed, costmap, collision_threshold)) {
    error = "neither the smoothed nor unsmoothed planner path is collision-free";
    return false;
  }

  const double start_yaw = tf2::getYaw(path.poses.front().pose.orientation);
  const double goal_yaw = tf2::getYaw(path.poses.back().pose.orientation);
  trajectory = generateTrajectory(smoothed, start_yaw, goal_yaw, limits_);
  if (trajectory.empty()) {
    error = "time-parameterized trajectory generation failed";
    return false;
  }
  return true;
}

void Nav2TrajectoryController::setPlan(const nav_msgs::msg::Path & path)
{
  resetReplanState();
  have_plan_ = false;
  trajectory_clock_started_ = false;
  original_path_.clear();
  trajectory_.clear();
  smoothed_path_.clear();
  actual_path_.poses.clear();
  CostmapGrid costmap;
  std::string costmap_frame;
  if (!snapshotGlobalCostmap(costmap, costmap_frame)) {
    throw nav2_core::PlannerException("Global costmap is unavailable or stale");
  }
  if (path.header.frame_id.empty() || path.header.frame_id != costmap_frame) {
    throw nav2_core::PlannerException(
            "Nav2 path frame does not match the global costmap frame");
  }

  std::vector<Point2> original_path;
  std::vector<Point2> smoothed;
  std::vector<TrajectoryPoint> trajectory;
  std::string error;
  if (!buildTrajectoryFromPath(
      path, costmap, costmap_frame, configured_waypoints_, blocked_cost_threshold_,
      original_path, smoothed, trajectory, error))
  {
    throw nav2_core::PlannerException(error);
  }
  original_path_ = std::move(original_path);
  smoothed_path_ = std::move(smoothed);
  trajectory_ = std::move(trajectory);
  route_waypoints_ = configured_waypoints_;
  waypoint_progress_ = 0;
  nearest_index_ = 0;
  completion_logged_ = false;
  squared_error_sum_ = 0.0;
  max_cross_track_error_ = 0.0;
  tracking_samples_ = 0;
  last_linear_command_ = 0.0;
  last_command_time_ = node_->now();
  trajectory_start_time_ = node_->now();
  trajectory_clock_started_ = true;
  actual_path_.poses.clear();
  have_plan_ = true;

  auto planned = path;
  planned.header.stamp = node_->now();
  planned_path_pub_->publish(planned);
  publishPath(smoothed_path_pub_, smoothed_path_, path.header.frame_id);
  publishTrajectory(path.header.frame_id, trajectory_start_time_);
  RCLCPP_INFO(
    node_->get_logger(),
    "BT FollowPath accepted %zu Nav2 poses, smoothed into %zu time-indexed trajectory samples",
    original_path_.size(), trajectory_.size());
}

void Nav2TrajectoryController::resetReplanState()
{
  std::lock_guard<std::mutex> lock(replan_mutex_);
  ++replan_generation_;
  replan_state_ = ReplanState::IDLE;
  replanned_path_ = nav_msgs::msg::Path{};
  replanned_waypoints_.clear();
  have_replan_attempt_ = false;
}

bool Nav2TrajectoryController::takeReplannedPath(
  nav_msgs::msg::Path & path, std::vector<Point2> & remaining_waypoints)
{
  std::lock_guard<std::mutex> lock(replan_mutex_);
  if (replan_state_ != ReplanState::READY) {
    return false;
  }
  path = std::move(replanned_path_);
  remaining_waypoints = std::move(replanned_waypoints_);
  replan_state_ = ReplanState::IDLE;
  return true;
}

bool Nav2TrajectoryController::replanIsPending() const
{
  std::lock_guard<std::mutex> lock(replan_mutex_);
  return replan_state_ == ReplanState::PENDING;
}

void Nav2TrajectoryController::requestReplan(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto now = node_->now();
  if (have_replan_attempt_ &&
    (now - last_replan_attempt_).seconds() < replan_retry_interval_)
  {
    return;
  }
  if (!planner_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Dynamic obstacle detected, but planner action '%s' is unavailable",
      planner_through_poses_action_.c_str());
    last_replan_attempt_ = now;
    have_replan_attempt_ = true;
    return;
  }

  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(replan_mutex_);
    if (replan_state_ == ReplanState::PENDING || replan_state_ == ReplanState::READY) {
      return;
    }
    replan_state_ = ReplanState::PENDING;
    generation = ++replan_generation_;
  }
  last_replan_attempt_ = now;
  have_replan_attempt_ = true;

  waypoint_progress_ = advanceWaypointProgress(
    {pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)},
    route_waypoints_, waypoint_progress_, std::max(goal_tolerance_, waypoint_match_tolerance_));
  std::vector<Point2> remaining_waypoints(
    route_waypoints_.begin() + static_cast<std::ptrdiff_t>(waypoint_progress_),
    route_waypoints_.end());
  if (remaining_waypoints.empty()) {
    remaining_waypoints.push_back({trajectory_.back().x, trajectory_.back().y});
  }

  ComputePath::Goal goal;
  goal.start = pose;
  goal.start.header.stamp = now;
  goal.goals.reserve(remaining_waypoints.size());
  for (std::size_t index = 0; index < remaining_waypoints.size(); ++index) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = pose.header.frame_id;
    waypoint.header.stamp = now;
    waypoint.pose.position.x = remaining_waypoints[index].x;
    waypoint.pose.position.y = remaining_waypoints[index].y;
    double yaw = trajectory_.back().heading;
    if (index + 1 < remaining_waypoints.size()) {
      yaw = std::atan2(
        remaining_waypoints[index + 1].y - remaining_waypoints[index].y,
        remaining_waypoints[index + 1].x - remaining_waypoints[index].x);
    }
    waypoint.pose.orientation = yawQuaternion(yaw);
    goal.goals.push_back(std::move(waypoint));
  }
  goal.planner_id = planner_id_;
  goal.use_start = true;

  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](const PlannerGoalHandle::SharedPtr & goal_handle) {
      if (goal_handle) {
        return;
      }
      std::lock_guard<std::mutex> lock(replan_mutex_);
      if (generation == replan_generation_ && replan_state_ == ReplanState::PENDING) {
        replan_state_ = ReplanState::FAILED;
      }
    };
  options.result_callback =
    [this, generation, remaining_waypoints](const PlannerGoalHandle::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(replan_mutex_);
      if (generation != replan_generation_ || replan_state_ != ReplanState::PENDING) {
        return;
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && !result.result->path.poses.empty())
      {
        replanned_path_ = result.result->path;
        replanned_waypoints_ = remaining_waypoints;
        replan_state_ = ReplanState::READY;
      } else {
        replan_state_ = ReplanState::FAILED;
      }
    };
  planner_client_->async_send_goal(goal, options);
  RCLCPP_WARN(
    node_->get_logger(), "Dynamic obstacle blocks the active trajectory; stopped and "
    "requesting a Nav2 path through %zu remaining ordered waypoint(s)",
    remaining_waypoints.size());
}

bool Nav2TrajectoryController::installReplannedPath(
  const nav_msgs::msg::Path & path, const CostmapGrid & costmap,
  const std::string & costmap_frame, const std::vector<Point2> & remaining_waypoints)
{
  std::vector<Point2> original;
  std::vector<Point2> smoothed;
  std::vector<TrajectoryPoint> trajectory;
  std::string error;
  if (!buildTrajectoryFromPath(
      path, costmap, costmap_frame, remaining_waypoints, dynamic_obstacle_cost_threshold_,
      original, smoothed, trajectory, error))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Rejecting unsafe dynamic replan; robot remains stopped: %s",
      error.c_str());
    return false;
  }

  original_path_ = std::move(original);
  smoothed_path_ = std::move(smoothed);
  trajectory_ = std::move(trajectory);
  route_waypoints_ = remaining_waypoints;
  waypoint_progress_ = 0;
  nearest_index_ = 0;
  completion_logged_ = false;
  squared_error_sum_ = 0.0;
  max_cross_track_error_ = 0.0;
  tracking_samples_ = 0;
  last_linear_command_ = 0.0;
  trajectory_start_time_ = node_->now();
  last_command_time_ = trajectory_start_time_;

  auto planned = path;
  planned.header.stamp = trajectory_start_time_;
  planned_path_pub_->publish(planned);
  publishPath(smoothed_path_pub_, smoothed_path_, path.header.frame_id);
  publishTrajectory(path.header.frame_id, trajectory_start_time_);
  RCLCPP_INFO(
    node_->get_logger(), "Installed collision-free Nav2 replan with %zu timed trajectory samples",
    trajectory_.size());
  return true;
}

void Nav2TrajectoryController::publishPath(
  const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr & publisher,
  const std::vector<Point2> & points, const std::string & frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = node_->now();
  path.poses.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto before = index == 0 ? index : index - 1;
    const auto after = std::min(index + 1, points.size() - 1);
    const double yaw = std::atan2(
      points[after].y - points[before].y, points[after].x - points[before].x);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[index].x;
    pose.pose.position.y = points[index].y;
    pose.pose.orientation = yawQuaternion(yaw);
    path.poses.push_back(std::move(pose));
  }
  publisher->publish(path);
}

void Nav2TrajectoryController::publishTrajectory(
  const std::string & frame, const rclcpp::Time & start_time)
{
  msg::Trajectory trajectory_message;
  trajectory_message.header.frame_id = frame;
  trajectory_message.header.stamp = start_time;
  trajectory_message.points.reserve(trajectory_.size());
  for (const auto & point : trajectory_) {
    msg::TrajectoryPoint sample;
    sample.x = point.x;
    sample.y = point.y;
    sample.heading = point.heading;
    sample.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start);
    sample.linear_velocity = point.linear_velocity;
    sample.angular_velocity = point.angular_velocity;
    trajectory_message.points.push_back(std::move(sample));
  }
  trajectory_pub_->publish(trajectory_message);
  publishPath(trajectory_path_pub_, smoothed_path_, frame);
}

void Nav2TrajectoryController::publishActualPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (actual_path_.poses.empty()) {
    actual_path_.poses.push_back(pose);
  } else {
    const auto & previous = actual_path_.poses.back().pose.position;
    const auto & current = pose.pose.position;
    if (std::hypot(current.x - previous.x, current.y - previous.y) >= 0.025) {
      actual_path_.poses.push_back(pose);
    }
  }
  actual_path_.header.frame_id = pose.header.frame_id;
  actual_path_.header.stamp = node_->now();
  actual_path_pub_->publish(actual_path_);
}

geometry_msgs::msg::TwistStamped Nav2TrajectoryController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  if (!have_plan_ || trajectory_.empty()) {
    throw nav2_core::PlannerException("Pure Pursuit has no valid trajectory");
  }
  publishActualPose(pose);

  Pose2 robot_pose;
  robot_pose.x = pose.pose.position.x;
  robot_pose.y = pose.pose.position.y;
  robot_pose.yaw = tf2::getYaw(pose.pose.orientation);
  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  const auto current_time = node_->now();
  if (!trajectory_clock_started_ || current_time < trajectory_start_time_) {
    trajectory_start_time_ = current_time;
    trajectory_clock_started_ = true;
    last_command_time_ = current_time;
  }
  const auto stopAndPauseTrajectory = [&]() {
      if (current_time >= last_command_time_) {
        trajectory_start_time_ = trajectory_start_time_ + (current_time - last_command_time_);
      } else {
        trajectory_start_time_ = current_time;
      }
      last_linear_command_ = 0.0;
      last_command_time_ = current_time;
    };

  CostmapGrid costmap;
  std::string costmap_frame;
  if (!snapshotGlobalCostmap(costmap, costmap_frame)) {
    stopAndPauseTrajectory();
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Global costmap is stale; stopping until obstacle data is available");
    return command;
  }

  nav_msgs::msg::Path replanned_path;
  std::vector<Point2> replanned_waypoints;
  if (takeReplannedPath(replanned_path, replanned_waypoints)) {
    if (!installReplannedPath(
        replanned_path, costmap, costmap_frame, replanned_waypoints))
    {
      stopAndPauseTrajectory();
      return command;
    }
  }

  waypoint_progress_ = advanceWaypointProgress(
    robot_pose, route_waypoints_, waypoint_progress_,
    std::max(goal_tolerance_, waypoint_match_tolerance_));
  const auto elapsed_time = (current_time - trajectory_start_time_).seconds();
  const auto first_remaining = std::lower_bound(
    trajectory_.begin(), trajectory_.end(), elapsed_time,
    [](const TrajectoryPoint & point, double time) {
      return point.time_from_start < time;
    });
  std::vector<Point2> remaining_path;
  remaining_path.reserve(
    static_cast<std::size_t>(std::distance(first_remaining, trajectory_.end())) + 1U);
  remaining_path.push_back({robot_pose.x, robot_pose.y});
  for (auto point = first_remaining; point != trajectory_.end(); ++point) {
    remaining_path.push_back({point->x, point->y});
  }
  if (!pathIsCollisionFree(
      remaining_path, costmap, dynamic_obstacle_cost_threshold_))
  {
    requestReplan(pose);
    stopAndPauseTrajectory();
    return command;
  }
  if (replanIsPending()) {
    stopAndPauseTrajectory();
    return command;
  }

  const auto & goal = trajectory_.back();
  const auto goal_command = goalAlignmentCommand(
    robot_pose, goal, goal_tolerance_, yaw_tolerance_, limits_.max_angular_velocity);
  if (goal_command.position_reached) {
    command.twist.linear.x = 0.0;
    command.twist.angular.z = goal_command.angular_velocity;
    last_linear_command_ = 0.0;
    last_command_time_ = node_->now();
    if (!completion_logged_ && goal_command.goal_reached) {
      completion_logged_ = true;
      const double rms = std::sqrt(
        squared_error_sum_ / static_cast<double>(std::max<std::size_t>(1, tracking_samples_)));
      RCLCPP_INFO(
        node_->get_logger(),
        "BT waypoint route reached tolerance: RMS/max tracking error %.3f/%.3f m, "
        "final position error %.3f m; zero command issued",
        rms, max_cross_track_error_,
        std::hypot(goal.x - robot_pose.x, goal.y - robot_pose.y));
    }
    return command;
  }

  const auto output = timeIndexedTrackingCommand(
    robot_pose, trajectory_, elapsed_time, velocity.linear.x, velocity.angular.z,
    current_max_linear_velocity_, limits_.max_angular_velocity);
  if (!output.valid) {
    throw nav2_core::PlannerException(
            "Time-indexed controller received invalid pose, timing, velocity, or trajectory data");
  }
  nearest_index_ = output.nearest_index;
  squared_error_sum_ += output.tracking_error * output.tracking_error;
  max_cross_track_error_ = std::max(max_cross_track_error_, output.tracking_error);
  ++tracking_samples_;

  command.twist.linear.x = output.linear_velocity;
  const double dt = std::clamp((node_->now() - last_command_time_).seconds(), 0.0, 0.2);
  const double measured_linear_velocity = std::isfinite(velocity.linear.x) ?
    std::max(0.0, velocity.linear.x) : last_linear_command_;
  const double acceleration_reference = std::clamp(
    std::max(last_linear_command_, measured_linear_velocity), 0.0, current_max_linear_velocity_);
  const double max_delta = limits_.max_linear_acceleration * dt;
  const double minimum_linear_velocity = std::max(0.0, acceleration_reference - max_delta);
  const double maximum_linear_velocity = std::min(
    current_max_linear_velocity_, acceleration_reference + max_delta);
  command.twist.linear.x = std::clamp(
    command.twist.linear.x, minimum_linear_velocity, maximum_linear_velocity);
  command.twist.angular.z = std::clamp(
    output.angular_velocity, -limits_.max_angular_velocity, limits_.max_angular_velocity);
  last_linear_command_ = command.twist.linear.x;
  last_command_time_ = node_->now();
  return command;
}

void Nav2TrajectoryController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    current_max_linear_velocity_ = base_max_linear_velocity_;
    return;
  }
  current_max_linear_velocity_ = percentage ?
    base_max_linear_velocity_ * std::min(speed_limit, 100.0) / 100.0 :
    std::min(speed_limit, base_max_linear_velocity_);
}

}  // namespace path_smoothing_controller

PLUGINLIB_EXPORT_CLASS(path_smoothing_controller::Nav2TrajectoryController, nav2_core::Controller)
