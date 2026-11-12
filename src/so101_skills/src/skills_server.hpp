#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <so101_interfaces/action/find_object.hpp>
#include <so101_interfaces/action/move_to.hpp>
#include <so101_interfaces/action/pick.hpp>
#include <so101_interfaces/action/place.hpp>
#include <so101_interfaces/srv/detect_objects.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace so101_skills
{
using FindObject = so101_interfaces::action::FindObject;
using MoveTo = so101_interfaces::action::MoveTo;
using Pick = so101_interfaces::action::Pick;
using Place = so101_interfaces::action::Place;
using DetectObjects = so101_interfaces::srv::DetectObjects;
using moveit::planning_interface::MoveGroupInterface;

inline constexpr char kArmGroup[] = "arm";
inline constexpr char kGripperGroup[] = "gripper";
inline constexpr char kTipLink[] = "gripper_frame_link";
inline constexpr char kCameraFrame[] = "wrist_camera_optical_frame";
inline const std::vector<std::string> kTouchLinks{ "gripper_link", "moving_jaw_so101_v1_link", "gripper_frame_link" };

struct Failure : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct Box
{
  Eigen::Isometry3d pose;  // centre
  Eigen::Vector3d size;
};

// The object in the gripper (from Pick to Place)
struct Grasp
{
  std::string id;
  double quarters;  // gripper yaw - object yaw in quarter turns
  double width;     // across the jaws
  double height;
};

class SkillsServer
{
public:
  explicit SkillsServer(const rclcpp::Node::SharedPtr& node);
  ~SkillsServer();

private:
  void findObject(const FindObject::Goal& goal, FindObject::Result& result);
  void pick(const Pick::Goal& goal, Pick::Result& result);
  void place(const Place::Goal& goal, Place::Result& result);
  void moveToNamed(const MoveTo::Goal& goal, MoveTo::Result& result);

  std::optional<std::vector<double>> watchJoints(const Eigen::Vector3d& target);
  void lookAndDetect(const std::string& id, const std::optional<std::vector<double>>& watch);

  double param(const std::string& name, double default_value);
  std::optional<double> optionalParam(const std::string& name);
  void throwIfCanceled(const std::string& what) const;
  void stage(const std::string& what);
  Box sceneObject(const std::string& id);
  Eigen::Isometry3d toPlanningFrame(geometry_msgs::msg::PoseStamped pose, const std::string& what);
  geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d& pose) const;

  geometry_msgs::msg::Pose downPose(double x, double y, double z, double yaw, double width) const;
  void attach(const std::string& id);
  void detach(const std::string& id);
  void moveTo(const geometry_msgs::msg::Pose& pose, const std::string& what);
  void gripperTo(const std::string& state);
  void planAndExecute(MoveGroupInterface& group, const std::string& what);
  void straightTo(const geometry_msgs::msg::Pose& pose, const std::string& what);
  void checkJointJumps(const trajectory_msgs::msg::JointTrajectory& traj, const std::string& what) const;
  void logToolError(const geometry_msgs::msg::Pose& target);

  template <class ActionT>
  using Run = std::function<void(const typename ActionT::Goal&, typename ActionT::Result&)>;
  template <class ActionT>
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

  template <class ActionT>
  std::shared_ptr<void> serve(const std::string& name, Run<ActionT> run)
  {
    return rclcpp_action::create_server<ActionT>(
        node_, name,
        [this, name](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionT::Goal>) {
          bool idle = false;
          if (!busy_.compare_exchange_strong(idle, true))
          {
            RCLCPP_WARN(log_, "%s: busy with another goal: rejected", name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // Only one goal runs at a time
        [this](const std::shared_ptr<Handle<ActionT>>&) {
          arm_.stop();
          gripper_.stop();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, name, run](const std::shared_ptr<Handle<ActionT>>& goal_handle) {
          if (worker_.joinable())
            worker_.join();  // the previous goal has finished (busy_ was false)
          worker_ = std::thread([this, name, run, goal_handle] { execute<ActionT>(name, run, goal_handle); });
        });
  }

  template <class ActionT>
  void execute(const std::string& name, const Run<ActionT>& run, const std::shared_ptr<Handle<ActionT>>& goal_handle)
  {
    canceling_ = [goal_handle] { return goal_handle->is_canceling(); };
    publish_stage_ = [goal_handle](const std::string& stage) {
      auto feedback = std::make_shared<typename ActionT::Feedback>();
      feedback->stage = stage;
      goal_handle->publish_feedback(feedback);
    };
    auto result = std::make_shared<typename ActionT::Result>();
    try
    {
      run(*goal_handle->get_goal(), *result);
      RCLCPP_INFO(log_, "%s done: %s", name.c_str(), result->message.c_str());
      busy_ = false;
      goal_handle->succeed(result);
    }
    catch (const std::exception& e)
    {
      result->message = e.what();
      busy_ = false;
      if (goal_handle->is_canceling())
      {
        RCLCPP_WARN(log_, "%s canceled: %s", name.c_str(), e.what());
        goal_handle->canceled(result);
      }
      else
      {
        RCLCPP_ERROR(log_, "%s: %s", name.c_str(), e.what());
        goal_handle->abort(result);
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  MoveGroupInterface arm_;
  MoveGroupInterface gripper_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::shared_ptr<void>> servers_;
  std::thread worker_;
  std::atomic<bool> busy_{ false };
  // The goal in progress (worker thread only)
  std::function<bool()> canceling_;
  std::function<void(const std::string&)> publish_stage_;
  std::optional<Grasp> held_;

  double grasp_depth_, approach_, jaw_clearance_, yaw_offset_, velocity_scaling_, max_joint_step_;
  std::optional<double> gripper_grasp_;
  bool use_camera_ = false;
  std::vector<double> look_joints_;
  std::vector<double> look_pan_offsets_;
  size_t pan_index_ = 0;
  rclcpp::Client<DetectObjects>::SharedPtr detect_client_;
};
}  // namespace so101_skills
