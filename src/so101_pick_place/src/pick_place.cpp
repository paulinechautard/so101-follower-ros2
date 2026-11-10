#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <so101_interfaces/action/pick_place.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

using PickPlace = so101_interfaces::action::PickPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr char kArmGroup[] = "arm";
constexpr char kGripperGroup[] = "gripper";
constexpr char kTipLink[] = "gripper_frame_link";
const std::vector<std::string> kTouchLinks{ "gripper_link", "moving_jaw_so101_v1_link", "gripper_frame_link" };

struct Failure : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct Box
{
  Eigen::Isometry3d pose;  // centre
  Eigen::Vector3d size;
};

class PickPlaceServer
{
public:
  explicit PickPlaceServer(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , log_(node->get_logger())
    , arm_(node, kArmGroup)
    , gripper_(node, kGripperGroup)
    , tf_buffer_(node->get_clock())
    , tf_listener_(tf_buffer_)
  {
    grasp_depth_ = param("grasp_depth", 0.015);
    approach_ = param("approach_height", 0.06);
    jaw_clearance_ = param("jaw_clearance", 0.005);
    yaw_offset_ = param("yaw_offset", 0.0);
    velocity_scaling_ = param("velocity_scaling", 0.3);
    max_joint_step_ = param("max_joint_step", 0.15);
    gripper_grasp_ = optionalParam("gripper_grasp");

    for (auto* group : { &arm_, &gripper_ })
    {
      group->setMaxVelocityScalingFactor(velocity_scaling_);
      group->setMaxAccelerationScalingFactor(velocity_scaling_);
      group->setPlanningTime(5.0);
      group->setNumPlanningAttempts(4);
    }
    // pick_ik stops at 1 mm / 0.01 rad (kinematics.yaml): accept a bit more than that.
    arm_.setGoalPositionTolerance(0.002);
    arm_.setGoalOrientationTolerance(0.02);

    server_ = rclcpp_action::create_server<PickPlace>(
        node, "pick_place",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const PickPlace::Goal>) {
          bool idle = false;
          if (!busy_.compare_exchange_strong(idle, true))
          {
            RCLCPP_WARN(log_, "Busy with another goal: rejected");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle>&) {
          arm_.stop();
          gripper_.stop();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle>& goal_handle) {
          // The goal runs outside the executor: MoveIt's calls block until done.
          if (worker_.joinable())
            worker_.join();  // the previous goal has finished (busy_ was false)
          worker_ = std::thread([this, goal_handle] { execute(goal_handle); });
        });
    RCLCPP_INFO(log_, "Ready: send goals to the pick_place action");
  }

  ~PickPlaceServer()
  {
    if (worker_.joinable())
      worker_.join();
  }

private:
  // MoveIt needs automatically_declare_parameters_from_overrides on this node (it reads
  // robot_description_kinematics.* etc.), so a parameter set in a YAML file is already
  // declared, with the YAML's type. Accept integers too: `yaw_offset: 0` is a valid number.
  double param(const std::string& name, double default_value)
  {
    if (!node_->has_parameter(name))
      node_->declare_parameter(name, default_value);
    return *optionalParam(name);
  }

  std::optional<double> optionalParam(const std::string& name)
  {
    if (!node_->has_parameter(name))
      return std::nullopt;
    const auto p = node_->get_parameter(name);
    switch (p.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(p.as_int());
      default:
        throw std::invalid_argument("parameter '" + name + "' must be a number, got " + p.value_to_string());
    }
  }

  void execute(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    auto result = std::make_shared<PickPlace::Result>();
    try
    {
      run(goal_handle);
      RCLCPP_INFO(log_, "Done");
      goal_handle->succeed(result);
    }
    catch (const std::exception& e)
    {
      result->message = e.what();
      if (goal_handle->is_canceling())
      {
        RCLCPP_WARN(log_, "Canceled: %s", e.what());
        goal_handle->canceled(result);
      }
      else
      {
        RCLCPP_ERROR(log_, "%s", e.what());
        goal_handle->abort(result);
      }
    }
    busy_ = false;
  }

  void run(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    goal_handle_ = goal_handle;
    const auto& goal = *goal_handle->get_goal();
    RCLCPP_INFO(log_, "Goal: pick '%s'", goal.object_id.c_str());

    const Box object = findObject(goal.object_id);
    const Eigen::Isometry3d place = placePose(goal.place_pose);

    // Gripper yaw: the closest to facing the base (+ yaw_offset) among the 4 that line the
    // jaws up with the object's faces. The jaws close along the tool's x axis.
    const Eigen::Matrix3d& rotation = object.pose.linear();
    if (rotation(2, 2) < 0.99)
      throw Failure("'" + goal.object_id + "' is not upright: can only grasp from above");
    const double object_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const Eigen::Vector3d pick = object.pose.translation();
    const double facing = std::atan2(pick.y(), pick.x()) + yaw_offset_;
    const double quarters = std::round(std::remainder(facing - object_yaw, 2.0 * M_PI) / M_PI_2);
    const double grasp_yaw = object_yaw + quarters * M_PI_2;
    const double width = std::fmod(std::abs(quarters), 2.0) == 0.0 ? object.size.x() : object.size.y();
    // The object turns with the gripper: keep the grasp's yaw relative to the object.
    const double place_yaw = std::atan2(place.linear()(1, 0), place.linear()(0, 0)) + quarters * M_PI_2;

    const double half_height = object.size.z() / 2.0;
    if (grasp_depth_ >= object.size.z())
      throw Failure("grasp_depth must be less than the object's height");
    const double grasp_z = pick.z() + half_height - grasp_depth_;
    // Put down 2 mm high rather than pushing into the support.
    const double release_z = place.translation().z() + half_height - grasp_depth_ + 0.002;
    const Eigen::Vector3d drop = place.translation();

    auto at_pick = [&](double z) { return downPose(pick.x(), pick.y(), z, grasp_yaw, width); };
    auto at_place = [&](double z) { return downPose(drop.x(), drop.y(), z, place_yaw, width); };

    gripperTo("open");
    moveTo(at_pick(grasp_z + approach_), "above the object");
    straightTo(at_pick(grasp_z), "descend to grasp");
    logToolError(at_pick(grasp_z));
    attach(goal.object_id);
    gripperTo("closed");
    straightTo(at_pick(grasp_z + approach_), "lift");

    moveTo(at_place(release_z + approach_), "above the place spot");
    straightTo(at_place(release_z), "descend to place");
    gripperTo("open");
    detach(goal.object_id);
    straightTo(at_place(release_z + approach_), "retreat");

    stage("home");
    arm_.setNamedTarget("home");
    planAndExecute(arm_, "home");
  }

  void throwIfCanceled(const std::string& what) const
  {
    if (goal_handle_->is_canceling())
      throw Failure("canceled before: " + what);
  }

  void stage(const std::string& what)
  {
    throwIfCanceled(what);
    RCLCPP_INFO(log_, "Stage: %s", what.c_str());
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->stage = what;
    goal_handle_->publish_feedback(feedback);
  }

  Box findObject(const std::string& id)
  {
    if (!scene_.getAttachedObjects({ id }).empty())
    {
      RCLCPP_WARN(log_, "'%s' is still attached to the robot: detaching it", id.c_str());
      detach(id);
    }
    const auto objects = scene_.getObjects({ id });
    const auto it = objects.find(id);
    if (it == objects.end())
      throw Failure("no object '" + id + "' in the planning scene (is scene_publisher running?)");
    const auto& object = it->second;
    if (object.header.frame_id != arm_.getPlanningFrame())
      throw Failure("'" + id + "' is in frame '" + object.header.frame_id + "', expected '" +
                    arm_.getPlanningFrame() + "'");
    if (object.primitives.size() != 1 || object.primitives[0].type != shape_msgs::msg::SolidPrimitive::BOX)
      throw Failure("'" + id + "' must be a single box");

    Eigen::Isometry3d object_pose, box_pose;
    tf2::fromMsg(object.pose, object_pose);
    tf2::fromMsg(object.primitive_poses[0], box_pose);
    const auto& d = object.primitives[0].dimensions;
    return { object_pose * box_pose, Eigen::Vector3d(d[0], d[1], d[2]) };
  }

  Eigen::Isometry3d placePose(geometry_msgs::msg::PoseStamped pose)
  {
    const std::string frame = arm_.getPlanningFrame();
    if (pose.header.frame_id.empty())
      pose.header.frame_id = frame;
    if (pose.header.frame_id != frame)
    {
      try
      {
        pose.header.stamp = rclcpp::Time(0);  // latest
        pose = tf_buffer_.transform(pose, frame, tf2::durationFromSec(1.0));
      }
      catch (const tf2::TransformException& e)
      {
        throw Failure("place_pose: " + std::string(e.what()));
      }
    }
    Eigen::Isometry3d result;
    tf2::fromMsg(pose.pose, result);
    return result;
  }

  // Tool pose that grasps a box centred at (x, y, z), gripper pointing straight down
  // (tool z = -base z), turned by yaw about the vertical. The 5 joints can do that
  // anywhere in reach, with any yaw (wrist_roll).
  //
  // gripper_frame_link is on the fixed jaw's inner face, and the moving jaw opens along
  // the tool's -x. So the tool frame goes half the object's width (plus a clearance)
  // along +x from the centre; closing pushes the object onto the fixed jaw.
  geometry_msgs::msg::Pose downPose(double x, double y, double z, double yaw, double width) const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()))
                        .toRotationMatrix();
    pose.translation() =
        Eigen::Vector3d(x, y, z) + pose.linear() * Eigen::Vector3d(width / 2.0 + jaw_clearance_, 0.0, 0.0);
    return tf2::toMsg(pose);
  }

  void attach(const std::string& id)
  {
    stage("attach");
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = kTipLink;
    attached.object.id = id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;  // the world object, by id
    attached.touch_links = kTouchLinks;
    if (!scene_.applyAttachedCollisionObject(attached))
      throw Failure("could not attach '" + id + "'");
  }

  void detach(const std::string& id)
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = kTipLink;
    attached.object.id = id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // back to the world
    if (!scene_.applyAttachedCollisionObject(attached))
      throw Failure("could not detach '" + id + "'");
  }

  // Joint-space move to a tool pose. move_group samples IK solutions for the goal and
  // keeps those that are collision-free.
  void moveTo(const geometry_msgs::msg::Pose& pose, const std::string& what)
  {
    stage(what);
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(pose, kTipLink);
    planAndExecute(arm_, what);
  }

  // "open", or "closed" (the SRDF's named states). Closing stalls the jaw on the object,
  // which gripper_controller reports as success (allow_stalling). gripper_grasp, when set,
  // replaces "closed": on the real arm, just past where the jaws touch the object.
  void gripperTo(const std::string& state)
  {
    stage(state == "open" ? "open the gripper" : "close the gripper");
    gripper_.setStartStateToCurrentState();
    if (state == "closed" && gripper_grasp_)
      gripper_.setJointValueTarget("gripper", *gripper_grasp_);
    else
      gripper_.setNamedTarget(state);
    planAndExecute(gripper_, "gripper " + state);
    RCLCPP_INFO(log_, "Gripper at %.3f rad", gripper_.getCurrentJointValues().at(0));
  }

  void planAndExecute(MoveGroupInterface& group, const std::string& what)
  {
    MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw Failure("planning failed: " + what);
    throwIfCanceled(what);  // the cancel may have arrived while planning
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw Failure("execution failed: " + what);
  }

  void straightTo(const geometry_msgs::msg::Pose& pose, const std::string& what)
  {
    stage(what);
    arm_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory msg;
    const double fraction = arm_.computeCartesianPath({ pose }, 0.005, msg, /*avoid_collisions=*/false);
    if (fraction < 0.99)
      throw Failure("Cartesian path " + what + " only " + std::to_string(fraction * 100) + "% feasible");

    const auto state = arm_.getCurrentState(2.0);
    if (!state)
      throw Failure("no current robot state");
    robot_trajectory::RobotTrajectory traj(arm_.getRobotModel(), kArmGroup);
    traj.setRobotTrajectoryMsg(*state, msg);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(traj, velocity_scaling_, velocity_scaling_))
      throw Failure("time parameterization failed: " + what);
    traj.getRobotTrajectoryMsg(msg);
    checkJointJumps(msg.joint_trajectory, what);

    throwIfCanceled(what);
    if (arm_.execute(msg) != moveit::core::MoveItErrorCode::SUCCESS)
      throw Failure("execution failed: " + what);
  }

  void checkJointJumps(const trajectory_msgs::msg::JointTrajectory& traj, const std::string& what) const
  {
    for (size_t i = 1; i < traj.points.size(); ++i)
    {
      const auto& prev = traj.points[i - 1].positions;
      const auto& cur = traj.points[i].positions;
      for (size_t j = 0; j < cur.size() && j < prev.size(); ++j)
      {
        const double step = std::abs(cur[j] - prev[j]);
        if (step > max_joint_step_)
          throw Failure("Cartesian path " + what + ": joint '" + traj.joint_names[j] + "' jumps " +
                        std::to_string(step) + " rad at waypoint " + std::to_string(i) + " (max_joint_step " +
                        std::to_string(max_joint_step_) + ")");
      }
    }
  }

  void logToolError(const geometry_msgs::msg::Pose& target)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let the arm settle
    const auto actual = arm_.getCurrentPose(kTipLink).pose.position;
    RCLCPP_INFO(log_, "Tool at (%.4f, %.4f, %.4f), target (%.4f, %.4f, %.4f): error %.1f mm", actual.x, actual.y,
                actual.z, target.position.x, target.position.y, target.position.z,
                1000.0 * std::hypot(actual.x - target.position.x, actual.y - target.position.y,
                                    actual.z - target.position.z));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  MoveGroupInterface arm_;
  MoveGroupInterface gripper_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<PickPlace>::SharedPtr server_;
  std::thread worker_;
  std::atomic<bool> busy_{ false };
  std::shared_ptr<GoalHandle> goal_handle_;

  double grasp_depth_, approach_, jaw_clearance_, yaw_offset_, velocity_scaling_, max_joint_step_;
  std::optional<double> gripper_grasp_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "pick_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spinning before MoveGroupInterface is constructed: it waits for move_group.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int code = 0;
  try
  {
    PickPlaceServer server(node);
    spinner.join();  // until Ctrl-C
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  if (spinner.joinable())
    spinner.join();
  return code;
}
