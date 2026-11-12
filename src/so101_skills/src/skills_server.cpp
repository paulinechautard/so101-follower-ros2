#include <algorithm>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "skills_server.hpp"

namespace so101_skills
{
SkillsServer::SkillsServer(const rclcpp::Node::SharedPtr& node)
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

  use_camera_ = node_->has_parameter("use_camera") && node_->get_parameter("use_camera").as_bool();
  if (use_camera_)
  {
    const auto joints = arm_.getActiveJoints();
    if (!node_->has_parameter("look_joints"))
      throw std::invalid_argument("use_camera needs the look_joints parameter");
    look_joints_ = node_->get_parameter("look_joints").as_double_array();
    if (look_joints_.size() != joints.size())
      throw std::invalid_argument("look_joints needs " + std::to_string(joints.size()) + " values");
    look_pan_offsets_ = { 0.0 };
    if (node_->has_parameter("look_pan_offsets"))
      look_pan_offsets_ = node_->get_parameter("look_pan_offsets").as_double_array();
    const auto pan = std::find(joints.begin(), joints.end(), std::string("shoulder_pan"));
    if (pan == joints.end())
      throw std::invalid_argument("no shoulder_pan joint in the arm group");
    pan_index_ = static_cast<size_t>(pan - joints.begin());
    if (!node_->has_parameter("detect_service"))
      node_->declare_parameter("detect_service", "/object_detector/detect");
    detect_client_ = node_->create_client<DetectObjects>(node_->get_parameter("detect_service").as_string());
  }

  for (auto* group : { &arm_, &gripper_ })
  {
    group->setMaxVelocityScalingFactor(velocity_scaling_);
    group->setMaxAccelerationScalingFactor(velocity_scaling_);
    group->setPlanningTime(5.0);
    group->setNumPlanningAttempts(4);
  }
  arm_.setGoalPositionTolerance(0.002);
  arm_.setGoalOrientationTolerance(0.02);

  servers_.push_back(serve<FindObject>("find_object", [this](const auto& goal, auto& result) {
    findObject(goal, result);
  }));
  servers_.push_back(serve<Pick>("pick", [this](const auto& goal, auto& result) { pick(goal, result); }));
  servers_.push_back(serve<Place>("place", [this](const auto& goal, auto& result) { place(goal, result); }));
  servers_.push_back(serve<MoveTo>("move_to", [this](const auto& goal, auto& result) { moveToNamed(goal, result); }));
  RCLCPP_INFO(log_, "Ready: actions find_object, pick, place and move_to");
}

SkillsServer::~SkillsServer()
{
  if (worker_.joinable())
    worker_.join();
}

double SkillsServer::param(const std::string& name, double default_value)
{
  if (!node_->has_parameter(name))
    node_->declare_parameter(name, default_value);
  return *optionalParam(name);
}

std::optional<double> SkillsServer::optionalParam(const std::string& name)
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

void SkillsServer::throwIfCanceled(const std::string& what) const
{
  if (canceling_())
    throw Failure("canceled before: " + what);
}

void SkillsServer::stage(const std::string& what)
{
  throwIfCanceled(what);
  RCLCPP_INFO(log_, "Stage: %s", what.c_str());
  publish_stage_(what);
}

Box SkillsServer::sceneObject(const std::string& id)
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

Eigen::Isometry3d SkillsServer::toPlanningFrame(geometry_msgs::msg::PoseStamped pose, const std::string& what)
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
      throw Failure(what + ": " + std::string(e.what()));
    }
  }
  Eigen::Isometry3d result;
  tf2::fromMsg(pose.pose, result);
  return result;
}

geometry_msgs::msg::PoseStamped SkillsServer::stamped(const Eigen::Isometry3d& pose) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = arm_.getPlanningFrame();
  msg.header.stamp = node_->now();
  msg.pose = tf2::toMsg(pose);
  return msg;
}
}  // namespace so101_skills
