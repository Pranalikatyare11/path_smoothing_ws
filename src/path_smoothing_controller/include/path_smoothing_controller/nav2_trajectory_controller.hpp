#ifndef TESTBED_NAVIGATION__NAV2_TRAJECTORY_CONTROLLER_HPP_
#define TESTBED_NAVIGATION__NAV2_TRAJECTORY_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "path_smoothing_controller/msg/trajectory.hpp"
#include "path_smoothing_controller/path_processing.hpp"
#include "tf2_ros/buffer.h"

namespace path_smoothing_controller
{

class Nav2TrajectoryController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using ComputePath = nav2_msgs::action::ComputePathThroughPoses;
  using PlannerGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

  void onGlobalCostmap(const nav2_msgs::msg::Costmap::SharedPtr message);
  bool snapshotGlobalCostmap(CostmapGrid & output, std::string & frame);
  bool buildTrajectoryFromPath(
    const nav_msgs::msg::Path & path, const CostmapGrid & costmap,
    const std::string & costmap_frame, const std::vector<Point2> & required_waypoints,
    std::uint8_t collision_threshold,
    std::vector<Point2> & original, std::vector<Point2> & smoothed,
    std::vector<TrajectoryPoint> & trajectory, std::string & error) const;
  bool installReplannedPath(
    const nav_msgs::msg::Path & path, const CostmapGrid & costmap,
    const std::string & costmap_frame, const std::vector<Point2> & remaining_waypoints);
  void requestReplan(const geometry_msgs::msg::PoseStamped & pose);
  bool takeReplannedPath(
    nav_msgs::msg::Path & path, std::vector<Point2> & remaining_waypoints);
  bool replanIsPending() const;
  void resetReplanState();
  void publishPath(
    const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    const std::vector<Point2> & points, const std::string & frame);
  void publishTrajectory(const std::string & frame, const rclcpp::Time & start_time);
  void publishActualPose(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  nav2_costmap_2d::Costmap2D * local_costmap_{nullptr};

  MotionLimits limits_;
  double base_max_linear_velocity_{0.26};
  double current_max_linear_velocity_{0.26};
  double lookahead_distance_{0.45};
  double waypoint_match_tolerance_{0.10};
  double goal_tolerance_{0.20};
  double yaw_tolerance_{0.20};
  double replan_retry_interval_{1.0};
  std::string planner_through_poses_action_{"/compute_path_through_poses"};
  std::string planner_id_{"GridBased"};
  int smoothing_iterations_{2};
  std::uint8_t blocked_cost_threshold_{253};
  std::uint8_t dynamic_obstacle_cost_threshold_{253};
  std::vector<Point2> configured_waypoints_;
  std::vector<Point2> route_waypoints_;
  std::size_t waypoint_progress_{0};

  std::vector<Point2> original_path_;
  std::vector<Point2> smoothed_path_;
  std::vector<TrajectoryPoint> trajectory_;
  std::size_t nearest_index_{0};
  nav_msgs::msg::Path actual_path_;
  bool have_plan_{false};
  bool completion_logged_{false};
  bool trajectory_clock_started_{false};
  double squared_error_sum_{0.0};
  double max_cross_track_error_{0.0};
  std::size_t tracking_samples_{0};
  double last_linear_command_{0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_attempt_{0, 0, RCL_ROS_TIME};
  bool have_replan_attempt_{false};

  CostmapGrid global_costmap_;
  std::string global_costmap_frame_;
  rclcpp::Time global_costmap_stamp_{0, 0, RCL_ROS_TIME};
  bool have_global_costmap_{false};
  std::mutex costmap_mutex_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  enum class ReplanState {IDLE, PENDING, READY, FAILED};
  mutable std::mutex replan_mutex_;
  ReplanState replan_state_{ReplanState::IDLE};
  nav_msgs::msg::Path replanned_path_;
  std::vector<Point2> replanned_waypoints_;
  std::uint64_t replan_generation_{0};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr trajectory_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<msg::Trajectory>::SharedPtr trajectory_pub_;
};

}  // namespace path_smoothing_controller

#endif  // TESTBED_NAVIGATION__NAV2_TRAJECTORY_CONTROLLER_HPP_
