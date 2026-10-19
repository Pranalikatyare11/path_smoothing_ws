#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "path_smoothing_controller/msg/trajectory.hpp"
#include "path_smoothing_controller/msg/trajectory_point.hpp"
#include "path_smoothing_controller/path_processing.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace path_smoothing_controller
{

class TrajectoryController : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using PlannerGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

  TrajectoryController()
  : Node("trajectory_controller"), tf_buffer_(get_clock())
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/trajectory_goal");
    waypoint_start_topic_ = declare_parameter<std::string>(
      "waypoint_start_topic", "/start_waypoint_list");
    waypoint_list_enabled_ = declare_parameter<bool>("waypoint_list_enabled", false);
    waypoint_goal_yaw_ = declare_parameter<double>("waypoint_goal_yaw", 0.0);
    const auto waypoint_values = declare_parameter<std::vector<double>>(
      "waypoints_xy", std::vector<double>{
        0.8, 5.0, 1.4, 5.3, 2.0, 5.0, 2.0, 4.4, 1.4, 4.2});
    configured_waypoints_ = parseWaypointsXY(waypoint_values);
    waypoint_list_valid_ = !configured_waypoints_.empty();
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    costmap_topic_ = declare_parameter<std::string>(
      "costmap_topic", "/global_costmap/costmap_raw");
    local_costmap_topic_ = declare_parameter<std::string>(
      "local_costmap_topic", "/local_costmap/costmap_raw");
    local_costmap_check_distance_ = declare_parameter<double>(
      "local_costmap_check_distance", 0.75);
    local_costmap_stale_timeout_ = declare_parameter<double>(
      "local_costmap_stale_timeout", 1.0);
    if (!std::isfinite(local_costmap_check_distance_) || local_costmap_check_distance_ <= 0.0 ||
      !std::isfinite(local_costmap_stale_timeout_) || local_costmap_stale_timeout_ <= 0.0)
    {
      throw std::invalid_argument(
              "local costmap check distance and stale timeout must be finite and positive");
    }
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    planner_action_ = declare_parameter<std::string>(
      "planner_action", "/compute_path_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");

    limits_.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 0.26);
    limits_.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 1.0);
    limits_.max_linear_acceleration = declare_parameter<double>("max_linear_acceleration", 0.5);
    max_angular_acceleration_ = declare_parameter<double>("max_angular_acceleration", 1.5);
    if (!std::isfinite(max_angular_acceleration_) || max_angular_acceleration_ <= 0.0) {
      throw std::invalid_argument("max_angular_acceleration must be finite and positive");
    }
    limits_.sample_spacing = declare_parameter<double>("sample_spacing", 0.05);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.45);
    smoothing_iterations_ = declare_parameter<int>("smoothing_iterations", 2);
    blocked_cost_threshold_ = static_cast<std::uint8_t>(
      declare_parameter<int>("blocked_cost_threshold", 253));
    const int dynamic_obstacle_threshold = static_cast<int>(
      declare_parameter<int>("dynamic_obstacle_cost_threshold", 253));
    dynamic_obstacle_cost_threshold_ = static_cast<std::uint8_t>(
      std::clamp(dynamic_obstacle_threshold, 1, 255));
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.20);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.20);
    tracking_failure_distance_ = declare_parameter<double>(
      "tracking_failure_distance", 1.25);
    tracking_failure_timeout_ = declare_parameter<double>(
      "tracking_failure_timeout", 3.0);
    replan_retry_interval_ = declare_parameter<double>("replan_retry_interval", 1.0);
    if (!std::isfinite(replan_retry_interval_) || replan_retry_interval_ <= 0.0) {
      throw std::invalid_argument("replan_retry_interval must be finite and positive");
    }

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    planner_client_ = rclcpp_action::create_client<ComputePath>(this, planner_action_);

    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10));
    planned_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/nav2_planned_path", rclcpp::QoS(1).transient_local());
    user_waypoints_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/user_waypoints", rclcpp::QoS(1).transient_local());
    smoothed_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/smoothed_path", rclcpp::QoS(1).transient_local());
    trajectory_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/trajectory_path", rclcpp::QoS(1).transient_local());
    trajectory_publisher_ = create_publisher<msg::Trajectory>(
      "/trajectory", rclcpp::QoS(1).transient_local());
    actual_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/actual_path", rclcpp::QoS(1));

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&TrajectoryController::onGoal, this, std::placeholders::_1));
    waypoint_start_subscription_ = create_subscription<std_msgs::msg::Empty>(
      waypoint_start_topic_, rclcpp::QoS(10),
      std::bind(&TrajectoryController::onStartWaypointList, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&TrajectoryController::onOdometry, this, std::placeholders::_1));
    costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(1),
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr message) {
        onCostmap(std::move(message), false);
      });
    local_costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
      local_costmap_topic_, rclcpp::QoS(1),
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr message) {
        onCostmap(std::move(message), true);
      });
    control_timer_ = create_wall_timer(
      50ms,
      std::bind(&TrajectoryController::controlStep, this));

    // Own /cmd_vel safely while this controller is active, including idle startup.
    publishStop();

    RCLCPP_INFO(
      get_logger(),
      "Custom controller ready and idle: goal=%s, waypoint_start=%s, waypoint_count=%zu, "
      "odom=%s, global_costmap=%s, local_costmap=%s, cmd_vel=%s; planner action=%s",
      goal_topic_.c_str(), waypoint_start_topic_.c_str(), configured_waypoints_.size(),
      odom_topic_.c_str(), costmap_topic_.c_str(), local_costmap_topic_.c_str(),
      cmd_vel_topic_.c_str(), planner_action_.c_str());
    if (!waypoint_list_valid_) {
      RCLCPP_ERROR(
        get_logger(), "waypoints_xy must contain at least five finite (x, y) pairs");
    }
  }

  ~TrajectoryController() override
  {
    publishStop();
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    last_odom_time_ = now();
    latest_odom_pose_.header = message->header;
    latest_odom_pose_.pose = message->pose.pose;
    have_odom_pose_ = true;
    const auto & twist = message->twist.twist;
    odom_linear_speed_ = std::hypot(twist.linear.x, twist.linear.y);
    odom_angular_speed_ = std::abs(twist.angular.z);
    odom_linear_velocity_ = twist.linear.x;
    odom_angular_velocity_ = twist.angular.z;
  }

  void onCostmap(const nav2_msgs::msg::Costmap::ConstSharedPtr message, bool local)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring %s costmap in frame '%s' (expected '%s')",
        local ? "local" : "global", message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    CostmapGrid grid;
    grid.width = message->metadata.size_x;
    grid.height = message->metadata.size_y;
    grid.resolution = message->metadata.resolution;
    grid.origin.x = message->metadata.origin.position.x;
    grid.origin.y = message->metadata.origin.position.y;
    grid.origin.yaw = tf2::getYaw(message->metadata.origin.orientation);
    grid.costs = message->data;
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (local) {
      local_costmap_ = std::move(grid);
      local_costmap_time_ = now();
      have_local_costmap_ = true;
    } else {
      costmap_ = std::move(grid);
      costmap_time_ = now();
      have_costmap_ = true;
    }
  }

  bool hasConflictingCommandPublisher()
  {
    const auto current_time = now();
    if (command_graph_checked_ && (current_time - last_command_graph_check_).seconds() < 0.5) {
      return command_publisher_conflict_;
    }
    command_graph_checked_ = true;
    last_command_graph_check_ = current_time;
    command_publisher_conflict_ = false;
    for (const auto & endpoint : get_publishers_info_by_topic(cmd_vel_topic_)) {
      if (endpoint.node_name() == get_name() && endpoint.node_namespace() == get_namespace()) {
        continue;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Another node (%s%s) publishes %s; refusing custom motion commands",
        endpoint.node_namespace().c_str(), endpoint.node_name().c_str(), cmd_vel_topic_.c_str());
      command_publisher_conflict_ = true;
      break;
    }
    return command_publisher_conflict_;
  }

  bool hasUniquePlannerActionServer()
  {
    std::string action_name;
    try {
      action_name = rclcpp::expand_topic_or_service_name(
        planner_action_, get_name(), get_namespace(), true);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Invalid planner action name '%s': %s",
        planner_action_.c_str(), error.what());
      return false;
    }

    const std::string goal_service = action_name + "/_action/send_goal";
    std::vector<std::string> servers;
    for (const auto & node : get_node_graph_interface()->get_node_names_and_namespaces()) {
      const auto services = get_service_names_and_types_by_node(node.first, node.second);
      if (services.count(goal_service) != 0U) {
        servers.push_back(node.second + "/" + node.first);
      }
    }

    if (servers.size() > 1U) {
      std::string server_list;
      for (const auto & server : servers) {
        if (!server_list.empty()) {
          server_list += ", ";
        }
        server_list += server;
      }
      RCLCPP_ERROR(
        get_logger(),
        "Found %zu servers for planner action '%s' (%s). Stop the extra planner_server; "
        "only one server may expose this action",
        servers.size(), action_name.c_str(), server_list.c_str());
      return false;
    }
    if (servers.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Could not identify the server node for '%s'; check `ros2 action info %s` if "
        "action responses are unmatched",
        action_name.c_str(), action_name.c_str());
    }
    return true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    ++goal_generation_;
    dynamic_replan_pending_ = false;
    have_replan_attempt_ = false;
    waypoint_sequence_active_ = false;
    planning_active_ = false;
    active_ = false;
    planned_segments_.clear();
    combined_plan_ = nav_msgs::msg::Path{};
    actual_path_.poses.clear();
    publishStop();

    if (hasConflictingCommandPublisher()) {
      return;
    }

    if (message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Goal frame must be '%s'; received '%s'",
        map_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }
    const auto & position = message->pose.position;
    const auto & orientation = message->pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-6)
    {
      RCLCPP_ERROR(get_logger(), "Ignoring a goal with invalid position or orientation");
      return;
    }
    if (!planner_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(
        get_logger(), "Nav2 planner action '%s' is not available",
        planner_action_.c_str());
      return;
    }
    if (!hasUniquePlannerActionServer()) {
      return;
    }
    active_waypoints_ = {{position.x, position.y}};
    waypoint_progress_ = 0;
    active_goal_yaw_ = tf2::getYaw(orientation);
    publishUserWaypoints();
    planning_active_ = true;
    RCLCPP_INFO(
      get_logger(), "Requesting a Nav2 path to (%.2f, %.2f)",
      position.x, position.y);
    requestPlannerGoal(*message, false, geometry_msgs::msg::PoseStamped{}, goal_generation_);
  }

  void onStartWaypointList(const std_msgs::msg::Empty::SharedPtr)
  {
    if (hasConflictingCommandPublisher()) {
      publishStop();
      return;
    }
    if (!waypoint_list_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "Waypoint-list mode is disabled; relaunch with waypoint_list_enabled:=true");
      publishStop();
      return;
    }
    if (!waypoint_list_valid_) {
      RCLCPP_ERROR(
        get_logger(), "Cannot start: waypoints_xy is not a valid list of five or more points");
      publishStop();
      return;
    }
    if (!planner_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "Cannot start waypoint list: Nav2 planner action is unavailable");
      publishStop();
      return;
    }
    if (!hasUniquePlannerActionServer()) {
      publishStop();
      return;
    }

    ++goal_generation_;
    dynamic_replan_pending_ = false;
    have_replan_attempt_ = false;
    active_ = false;
    planning_active_ = true;
    waypoint_sequence_active_ = true;
    waypoint_segment_index_ = 0;
    waypoint_progress_ = 0;
    active_waypoints_ = configured_waypoints_;
    active_goal_yaw_ = waypoint_goal_yaw_;
    planned_segments_.clear();
    combined_plan_ = nav_msgs::msg::Path{};
    actual_path_.poses.clear();
    publishStop();
    publishUserWaypoints();
    RCLCPP_INFO(
      get_logger(), "Starting explicit ordered waypoint sequence (%zu points)",
      active_waypoints_.size());
    requestWaypointSegment(0, goal_generation_, false);
  }

  double waypointYaw(std::size_t index) const
  {
    if (index + 1 < active_waypoints_.size()) {
      return std::atan2(
        active_waypoints_[index + 1].y - active_waypoints_[index].y,
        active_waypoints_[index + 1].x - active_waypoints_[index].x);
    }
    return active_goal_yaw_;
  }

  geometry_msgs::msg::PoseStamped makeWaypointPose(std::size_t index) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = active_waypoints_[index].x;
    pose.pose.position.y = active_waypoints_[index].y;
    pose.pose.orientation = quaternionFromYaw(waypointYaw(index));
    return pose;
  }

  void requestWaypointSegment(std::size_t index, std::uint64_t generation, bool use_start)
  {
    waypoint_segment_index_ = index;
    geometry_msgs::msg::PoseStamped start;
    if (use_start) {
      if (dynamic_replan_pending_ && index == 0) {
        start = dynamic_replan_start_;
      } else if (index > 0) {
        start = makeWaypointPose(index - 1);
      }
    }
    requestPlannerGoal(makeWaypointPose(index), use_start, start, generation);
  }

  void requestPlannerGoal(
    const geometry_msgs::msg::PoseStamped & goal, bool use_start,
    const geometry_msgs::msg::PoseStamped & start, std::uint64_t generation)
  {
    ComputePath::Goal planner_goal;
    planner_goal.goal = goal;
    planner_goal.start = start;
    planner_goal.planner_id = planner_id_;
    planner_goal.use_start = use_start;
    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, generation](const PlannerGoalHandle::SharedPtr & handle) {
        if (!handle && generation == goal_generation_) {
          if (dynamic_replan_pending_) {
            dynamic_replan_pending_ = false;
            planning_active_ = false;
            publishStop();
            RCLCPP_WARN(get_logger(), "Nav2 rejected the dynamic replan; will retry while stopped");
          } else {
            failPlanning("Nav2 rejected the path request");
          }
        }
      };
    options.result_callback = [this, generation](const PlannerGoalHandle::WrappedResult & result) {
        onPlanResult(generation, result);
      };
    planner_client_->async_send_goal(planner_goal, options);
  }

  void failPlanning(const std::string & reason)
  {
    if (dynamic_replan_pending_) {
      dynamic_replan_pending_ = false;
      planning_active_ = false;
      publishStop();
      RCLCPP_WARN(
        get_logger(), "Dynamic replan failed; keeping the robot stopped for retry: %s",
        reason.c_str());
      return;
    }
    planning_active_ = false;
    waypoint_sequence_active_ = false;
    active_ = false;
    publishStop();
    RCLCPP_ERROR(get_logger(), "Planning stopped: %s", reason.c_str());
  }

  void onPlanResult(
    std::uint64_t generation, const PlannerGoalHandle::WrappedResult & wrapped)
  {
    if (generation != goal_generation_) {
      return;
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      failPlanning("Nav2 ComputePathToPose did not return a path");
      return;
    }
    const auto & segment = wrapped.result->path;
    if (segment.poses.empty()) {
      failPlanning("Nav2 returned an empty path");
      return;
    }
    if (!segment.header.frame_id.empty() && segment.header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Nav2 path is in frame '%s', expected '%s'",
        segment.header.frame_id.c_str(), map_frame_.c_str());
      failPlanning("Nav2 path frame does not match the configured map frame");
      return;
    }

    if (!waypoint_sequence_active_) {
      planning_active_ = false;
      acceptPlannedPaths(segment, {segment});
      return;
    }

    auto normalized_segment = segment;
    if (!combined_plan_.poses.empty()) {
      auto & first = normalized_segment.poses.front();
      const auto & previous = combined_plan_.poses.back();
      const double join_error = std::hypot(
        first.pose.position.x - previous.pose.position.x,
        first.pose.position.y - previous.pose.position.y);
      if (join_error > 0.15) {
        failPlanning("Nav2 returned discontinuous paths at an ordered waypoint");
        return;
      }
      first.pose.position.x = previous.pose.position.x;
      first.pose.position.y = previous.pose.position.y;
      first.pose.orientation = previous.pose.orientation;
    } else {
      combined_plan_.header = segment.header;
    }

    planned_segments_.push_back(normalized_segment);
    const std::size_t first_pose = combined_plan_.poses.empty() ? 0U : 1U;
    combined_plan_.poses.insert(
      combined_plan_.poses.end(), normalized_segment.poses.begin() + first_pose,
      normalized_segment.poses.end());
    if (combined_plan_.header.frame_id.empty()) {
      combined_plan_.header.frame_id = map_frame_;
    }

    if (waypoint_segment_index_ + 1 < active_waypoints_.size()) {
      requestWaypointSegment(waypoint_segment_index_ + 1, generation, true);
      return;
    }

    planning_active_ = false;
    waypoint_sequence_active_ = false;
    acceptPlannedPaths(combined_plan_, planned_segments_);
  }

  void acceptPlannedPaths(
    const nav_msgs::msg::Path & path_message,
    const std::vector<nav_msgs::msg::Path> & path_segments)
  {
    if (path_message.poses.empty() || path_segments.empty()) {
      failPlanning("Nav2 returned an empty combined path");
      return;
    }

    CostmapGrid costmap;
    CostmapGrid local_costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!have_costmap_) {
        failPlanning("No global costmap received; refusing to track the path");
        return;
      }
      if ((now() - costmap_time_).seconds() > 2.0) {
        failPlanning("Global costmap is stale; refusing to track the path");
        return;
      }
      costmap = costmap_;
      if (!have_local_costmap_) {
        failPlanning("No local LiDAR costmap received; refusing to track the path");
        return;
      }
      if ((now() - local_costmap_time_).seconds() > local_costmap_stale_timeout_) {
        failPlanning("Local LiDAR costmap is stale; refusing to track the path");
        return;
      }
      local_costmap = local_costmap_;
    }

    std::vector<Point2> original;
    original.reserve(path_message.poses.size());
    for (const auto & pose : path_message.poses) {
      original.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    const auto collision_threshold = dynamic_replan_pending_ ?
      dynamic_obstacle_cost_threshold_ : blocked_cost_threshold_;
    if (!pathIsCollisionFree(original, costmap, collision_threshold)) {
      failPlanning("Nav2 path intersects a blocked/unknown cell or leaves the global costmap");
      return;
    }
    if (!pathIsCollisionFreeInCostmapWindow(
        original, local_costmap, dynamic_obstacle_cost_threshold_,
        local_costmap_check_distance_))
    {
      failPlanning("Nav2 path is blocked or leaves the checked local costmap window");
      return;
    }

    const double start_yaw = tf2::getYaw(path_segments.front().poses.front().pose.orientation);
    const double goal_yaw = tf2::getYaw(path_segments.back().poses.back().pose.orientation);
    std::vector<std::vector<Point2>> points_by_segment;
    points_by_segment.reserve(path_segments.size());
    for (const auto & path_segment : path_segments) {
      std::vector<Point2> points;
      points.reserve(path_segment.poses.size());
      for (const auto & pose : path_segment.poses) {
        points.push_back({pose.pose.position.x, pose.pose.position.y});
      }
      points_by_segment.push_back(std::move(points));
    }
    auto smoothed = smoothPathSegments(
      points_by_segment,
      static_cast<std::size_t>(std::max(0, smoothing_iterations_)), limits_.sample_spacing);
    if (smoothed.empty()) {
      failPlanning("Path smoothing failed");
      return;
    }
    if (!pathIsCollisionFree(smoothed, costmap, collision_threshold) ||
      !pathIsCollisionFreeInCostmapWindow(
        smoothed, local_costmap, dynamic_obstacle_cost_threshold_,
        local_costmap_check_distance_))
    {
      RCLCPP_WARN(
        get_logger(),
        "Smoothed path fails global or local costmap clearance; using the validated Nav2 path");
      smoothed = smoothPathSegments(points_by_segment, 0, limits_.sample_spacing);
    }
    if (!pathIsCollisionFree(smoothed, costmap, collision_threshold) ||
      !pathIsCollisionFreeInCostmapWindow(
        smoothed, local_costmap, dynamic_obstacle_cost_threshold_,
        local_costmap_check_distance_))
    {
      failPlanning("Neither smoothed nor original path passes collision checks");
      return;
    }

    auto trajectory = generateTrajectory(smoothed, start_yaw, goal_yaw, limits_);
    if (trajectory.empty()) {
      failPlanning("Trajectory generation failed; refusing to move");
      return;
    }

    dynamic_replan_pending_ = false;
    have_replan_attempt_ = false;
    const auto trajectory_start_time = now();
    publishPlannedPath(path_message);
    publishUserWaypoints();
    publishSmoothedPath(smoothed);
    publishTrajectory(trajectory, trajectory_start_time);
    trajectory_ = std::move(trajectory);
    goal_x_ = path_message.poses.back().pose.position.x;
    goal_y_ = path_message.poses.back().pose.position.y;
    goal_yaw_ = goal_yaw;
    nearest_index_ = 0;
    waypoint_progress_ = 0;
    cross_track_started_ = false;
    squared_error_sum_ = 0.0;
    max_cross_track_error_ = 0.0;
    tracking_samples_ = 0;
    active_ = true;
    last_control_time_ = trajectory_start_time;
    trajectory_start_time_ = trajectory_start_time;
    last_tf_time_ = now();
    last_linear_command_ = 0.0;
    RCLCPP_INFO(
      get_logger(),
      "Following %zu trajectory samples; Nav2 path=%zu poses across %zu leg(s), "
      "smoothed path=%zu samples, user waypoints=%zu",
      trajectory_.size(), original.size(), path_segments.size(), smoothed.size(),
      active_waypoints_.size());
  }

  void publishPlannedPath(const nav_msgs::msg::Path & input)
  {
    auto path = input;
    if (path.header.frame_id.empty()) {
      path.header.frame_id = map_frame_;
    }
    path.header.stamp = now();
    planned_path_publisher_->publish(path);
  }

  void publishUserWaypoints()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (std::size_t i = 0; i < active_waypoints_.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = active_waypoints_[i].x;
      pose.pose.position.y = active_waypoints_[i].y;
      pose.pose.orientation = quaternionFromYaw(waypointYaw(i));
      path.poses.push_back(std::move(pose));
    }
    user_waypoints_publisher_->publish(path);
  }

  nav_msgs::msg::Path makePathMessage(
    const std::vector<TrajectoryPoint> & trajectory, bool add_time)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    const rclcpp::Time start_time(path.header.stamp);
    path.poses.reserve(trajectory.size());
    for (const auto & point : trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = add_time ?
        static_cast<builtin_interfaces::msg::Time>(
        start_time + rclcpp::Duration::from_seconds(point.time_from_start)) :
        path.header.stamp;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = quaternionFromYaw(point.heading);
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) const
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(quaternion);
  }

  void publishSmoothedPath(const std::vector<Point2> & points)
  {
    std::vector<TrajectoryPoint> path_points;
    path_points.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      double heading = 0.0;
      if (points.size() > 1) {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after = i + 1 < points.size() ? i + 1 : points.size() - 1;
        heading = std::atan2(
          points[after].y - points[before].y,
          points[after].x - points[before].x);
      }
      path_points.push_back({points[i].x, points[i].y, heading, 0.0, 0.0, 0.0});
    }
    smoothed_path_publisher_->publish(makePathMessage(path_points, false));
  }

  void publishTrajectory(
    const std::vector<TrajectoryPoint> & points, const rclcpp::Time & start_time)
  {
    msg::Trajectory trajectory_message;
    trajectory_message.header.frame_id = map_frame_;
    trajectory_message.header.stamp = start_time;
    trajectory_message.points.reserve(points.size());
    for (const auto & point : points) {
      msg::TrajectoryPoint sample;
      sample.x = point.x;
      sample.y = point.y;
      sample.heading = point.heading;
      sample.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(point.time_from_start));
      sample.linear_velocity = point.linear_velocity;
      sample.angular_velocity = point.angular_velocity;
      trajectory_message.points.push_back(sample);
    }
    trajectory_publisher_->publish(trajectory_message);
    trajectory_path_publisher_->publish(makePathMessage(points, true));
  }

  bool poseFromOdometry(Pose2 & current)
  {
    if (!have_odom_pose_) {
      return false;
    }
    const std::string source_frame = latest_odom_pose_.header.frame_id.empty() ?
      odom_frame_ : latest_odom_pose_.header.frame_id;
    try {
      const auto map_from_odom_message = tf_buffer_.lookupTransform(
        map_frame_, source_frame, tf2::TimePointZero);
      tf2::Transform map_from_odom;
      tf2::Transform odom_from_base;
      tf2::fromMsg(map_from_odom_message.transform, map_from_odom);
      tf2::fromMsg(latest_odom_pose_.pose, odom_from_base);
      const auto map_from_base = map_from_odom * odom_from_base;
      current.x = map_from_base.getOrigin().x();
      current.y = map_from_base.getOrigin().y();
      current.yaw = tf2::getYaw(map_from_base.getRotation());
      last_tf_time_ = now();
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s to %s transform: %s",
        map_frame_.c_str(), source_frame.c_str(), error.what());
      return false;
    }
  }

  void publishActualPath()
  {
    Pose2 current;
    if (!poseFromOdometry(current)) {
      return;
    }
    if (actual_path_.poses.empty() || std::hypot(
        current.x - actual_path_.poses.back().pose.position.x,
        current.y - actual_path_.poses.back().pose.position.y) > 0.025)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.header.stamp = now();
      pose.pose.position.x = current.x;
      pose.pose.position.y = current.y;
      pose.pose.orientation = quaternionFromYaw(current.yaw);
      actual_path_.poses.push_back(std::move(pose));
    }
    actual_path_.header.frame_id = map_frame_;
    actual_path_.header.stamp = now();
    actual_path_publisher_->publish(actual_path_);
  }

  void requestDynamicReplan(const Pose2 & robot_pose, const std::string & reason)
  {
    const auto current_time = now();
    if (have_replan_attempt_ &&
      (current_time - last_replan_attempt_).seconds() < replan_retry_interval_)
    {
      return;
    }
    if (!planner_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dynamic obstacle detected, but Nav2 planner action '%s' is unavailable",
        planner_action_.c_str());
      last_replan_attempt_ = current_time;
      have_replan_attempt_ = true;
      return;
    }

    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = map_frame_;
    start.header.stamp = current_time;
    start.pose.position.x = robot_pose.x;
    start.pose.position.y = robot_pose.y;
    start.pose.orientation = quaternionFromYaw(robot_pose.yaw);

    waypoint_progress_ = advanceWaypointProgress(
      robot_pose, active_waypoints_, waypoint_progress_,
      std::max(goal_tolerance_, 0.0));
    if (waypoint_progress_ >= active_waypoints_.size()) {
      waypoint_progress_ = active_waypoints_.size() - 1;
    }
    active_waypoints_ = std::vector<Point2>(
      active_waypoints_.begin() + static_cast<std::ptrdiff_t>(waypoint_progress_),
      active_waypoints_.end());
    waypoint_progress_ = 0;
    if (active_waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "Cannot replan: no remaining waypoint goals");
      return;
    }

    ++goal_generation_;
    dynamic_replan_pending_ = true;
    planning_active_ = true;
    waypoint_sequence_active_ = true;
    waypoint_segment_index_ = 0;
    dynamic_replan_start_ = start;
    planned_segments_.clear();
    combined_plan_ = nav_msgs::msg::Path{};
    last_replan_attempt_ = current_time;
    have_replan_attempt_ = true;
    publishUserWaypoints();
    requestWaypointSegment(0, goal_generation_, true);
    RCLCPP_WARN(
      get_logger(),
      "Trajectory safety check failed (%s); stopped and replanning through %zu "
      "remaining ordered waypoint(s)",
      reason.c_str(), active_waypoints_.size());
  }

  void controlStep()
  {
    publishActualPath();
    if (hasConflictingCommandPublisher()) {
      if (active_) {
        abortTracking("another node is publishing on the configured cmd_vel topic");
      } else {
        publishStop();
      }
      return;
    }
    if (!active_) {
      publishStop();
      return;
    }
    const auto current_time = now();
    if ((current_time - last_odom_time_).seconds() > 1.0) {
      abortTracking("odometry timed out");
      return;
    }
    if ((current_time - last_tf_time_).seconds() > 1.0) {
      abortTracking("map-to-base TF timed out");
      return;
    }

    Pose2 robot_pose;
    if (!poseFromOdometry(robot_pose)) {
      return;
    }
    waypoint_progress_ = advanceWaypointProgress(
      robot_pose, active_waypoints_, waypoint_progress_,
      std::max(goal_tolerance_, 0.0));

    if (current_time < trajectory_start_time_) {
      trajectory_start_time_ = current_time;
      last_control_time_ = current_time;
    }
    const auto stopAndPauseTrajectory = [&]() {
        if (current_time >= last_control_time_) {
          trajectory_start_time_ = trajectory_start_time_ + (current_time - last_control_time_);
        } else {
          trajectory_start_time_ = current_time;
        }
        last_linear_command_ = 0.0;
        last_control_time_ = current_time;
        publishStop();
      };
    CostmapGrid current_costmap;
    CostmapGrid current_local_costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!have_costmap_ || (current_time - costmap_time_).seconds() > 2.0) {
        stopAndPauseTrajectory();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Global costmap is missing or stale; stopping until obstacle data is available");
        return;
      }
      current_costmap = costmap_;
      if (!have_local_costmap_ ||
        (current_time - local_costmap_time_).seconds() > local_costmap_stale_timeout_)
      {
        stopAndPauseTrajectory();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Local LiDAR costmap is missing or stale; stopping until obstacle data is available");
        return;
      }
      current_local_costmap = local_costmap_;
    }
    if (dynamic_replan_pending_) {
      stopAndPauseTrajectory();
      return;
    }

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
        remaining_path, current_costmap, dynamic_obstacle_cost_threshold_))
    {
      requestDynamicReplan(
        robot_pose, "global costmap marks the remaining path blocked, unknown, or out of bounds");
      stopAndPauseTrajectory();
      return;
    }
    if (!pathIsCollisionFreeInCostmapWindow(
        remaining_path, current_local_costmap, dynamic_obstacle_cost_threshold_,
        local_costmap_check_distance_))
    {
      requestDynamicReplan(
        robot_pose, "local costmap marks the nearby path blocked or outside its rolling window");
      stopAndPauseTrajectory();
      return;
    }
    const auto output = timeIndexedTrackingCommand(
      robot_pose, trajectory_, elapsed_time,
      odom_linear_velocity_, odom_angular_velocity_,
      limits_.max_linear_velocity, limits_.max_angular_velocity);
    if (!output.valid) {
      abortTracking("controller received invalid trajectory timing, pose, or odometry");
      return;
    }
    nearest_index_ = output.nearest_index;
    squared_error_sum_ += output.tracking_error * output.tracking_error;
    max_cross_track_error_ = std::max(max_cross_track_error_, output.tracking_error);
    ++tracking_samples_;

    if (output.tracking_error > tracking_failure_distance_) {
      if (!cross_track_started_) {
        cross_track_started_ = true;
        cross_track_start_time_ = current_time;
      } else if ((current_time - cross_track_start_time_).seconds() >
        tracking_failure_timeout_)
      {
        abortTracking("trajectory tracking error remained above the failure threshold");
        return;
      }
    } else {
      cross_track_started_ = false;
    }

    const double goal_distance = std::hypot(goal_x_ - robot_pose.x, goal_y_ - robot_pose.y);
    const double goal_yaw_error = normalizeAngle(goal_yaw_ - robot_pose.yaw);
    if (goal_distance <= goal_tolerance_ && std::abs(goal_yaw_error) <= yaw_tolerance_ &&
      odom_linear_speed_ < 0.05 && odom_angular_speed_ < 0.10)
    {
      active_ = false;
      publishStop();
      const double rms = tracking_samples_ == 0 ? 0.0 :
        std::sqrt(squared_error_sum_ / static_cast<double>(tracking_samples_));
      RCLCPP_INFO(
        get_logger(),
        "Goal complete. RMS tracking error=%.3f m, max=%.3f m, final position error=%.3f m",
        rms, max_cross_track_error_, goal_distance);
      return;
    }

    geometry_msgs::msg::Twist command;
    if (goal_distance <= goal_tolerance_ && std::abs(goal_yaw_error) > yaw_tolerance_) {
      command.angular.z = std::clamp(
        1.5 * goal_yaw_error,
        -limits_.max_angular_velocity, limits_.max_angular_velocity);
    } else {
      command.linear.x = output.linear_velocity;
      command.angular.z = output.angular_velocity;
    }
    const double dt = std::clamp((current_time - last_control_time_).seconds(), 0.0, 0.2);
    command.linear.x = limitVelocityChange(
      std::clamp(command.linear.x, 0.0, limits_.max_linear_velocity),
      last_linear_command_, limits_.max_linear_acceleration, dt);
    command.angular.z = std::clamp(
      command.angular.z,
      -limits_.max_angular_velocity, limits_.max_angular_velocity);
    command.angular.z = limitVelocityChange(
      command.angular.z, last_angular_command_, max_angular_acceleration_, dt);
    last_linear_command_ = command.linear.x;
    last_angular_command_ = command.angular.z;
    last_control_time_ = current_time;
    cmd_publisher_->publish(command);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_publisher_->publish(stop);
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
  }

  void abortTracking(const std::string & reason)
  {
    active_ = false;
    publishStop();
    RCLCPP_ERROR(get_logger(), "Stopping custom tracking: %s", reason.c_str());
  }

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr user_waypoints_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_path_publisher_;
  rclcpp::Publisher<msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr waypoint_start_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::mutex costmap_mutex_;
  CostmapGrid costmap_;
  CostmapGrid local_costmap_;
  bool have_costmap_{false};
  bool have_local_costmap_{false};
  std::vector<TrajectoryPoint> trajectory_;
  std::vector<Point2> configured_waypoints_;
  std::vector<Point2> active_waypoints_;
  std::size_t waypoint_progress_{0};
  std::vector<nav_msgs::msg::Path> planned_segments_;
  nav_msgs::msg::Path combined_plan_;
  nav_msgs::msg::Path actual_path_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseStamped dynamic_replan_start_;
  MotionLimits limits_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string goal_topic_;
  std::string waypoint_start_topic_;
  std::string odom_topic_;
  std::string costmap_topic_;
  std::string local_costmap_topic_;
  std::string cmd_vel_topic_;
  std::string planner_action_;
  std::string planner_id_;
  std::uint8_t blocked_cost_threshold_{253};
  std::uint8_t dynamic_obstacle_cost_threshold_{253};
  int smoothing_iterations_{2};
  double lookahead_distance_{0.45};
  double goal_tolerance_{0.20};
  double yaw_tolerance_{0.20};
  double tracking_failure_distance_{1.25};
  double tracking_failure_timeout_{3.0};
  double replan_retry_interval_{1.0};
  double max_angular_acceleration_{1.5};
  double local_costmap_check_distance_{0.75};
  double local_costmap_stale_timeout_{1.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double waypoint_goal_yaw_{0.0};
  double active_goal_yaw_{0.0};
  double odom_linear_speed_{0.0};
  double odom_angular_speed_{0.0};
  double odom_linear_velocity_{0.0};
  double odom_angular_velocity_{0.0};
  double last_linear_command_{0.0};
  double last_angular_command_{0.0};
  double squared_error_sum_{0.0};
  double max_cross_track_error_{0.0};
  rclcpp::Time costmap_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_costmap_time_{0, 0, RCL_ROS_TIME};
  std::size_t nearest_index_{0};
  std::size_t tracking_samples_{0};
  std::size_t waypoint_segment_index_{0};
  std::uint64_t goal_generation_{0};
  bool active_{false};
  bool planning_active_{false};
  bool waypoint_sequence_active_{false};
  bool waypoint_list_enabled_{false};
  bool waypoint_list_valid_{false};
  bool have_odom_pose_{false};
  bool cross_track_started_{false};
  bool dynamic_replan_pending_{false};
  bool have_replan_attempt_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tf_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cross_track_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_attempt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_graph_check_{0, 0, RCL_ROS_TIME};
  bool command_graph_checked_{false};
  bool command_publisher_conflict_{false};
};

}  // namespace path_smoothing_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_smoothing_controller::TrajectoryController>());
  rclcpp::shutdown();
  return 0;
}
