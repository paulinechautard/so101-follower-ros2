#include <chrono>
#include <cmath>
#include <thread>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "skills_server.hpp"

namespace so101_skills
{
// Tool pose that grasps a box centred at (x, y, z)
geometry_msgs::msg::Pose SkillsServer::downPose(double x, double y, double z, double yaw, double width) const
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()))
                      .toRotationMatrix();
  pose.translation() =
      Eigen::Vector3d(x, y, z) + pose.linear() * Eigen::Vector3d(width / 2.0 + jaw_clearance_, 0.0, 0.0);
  return tf2::toMsg(pose);
}

void SkillsServer::attach(const std::string& id)
{
  stage("attach");
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = kTipLink;
  attached.object.id = id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;  // the world object by id
  attached.touch_links = kTouchLinks;
  if (!scene_.applyAttachedCollisionObject(attached))
    throw Failure("could not attach '" + id + "'");
}

void SkillsServer::detach(const std::string& id)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = kTipLink;
  attached.object.id = id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // back to the world
  if (!scene_.applyAttachedCollisionObject(attached))
    throw Failure("could not detach '" + id + "'");
}

// Planned around collisions unlike straightTo
void SkillsServer::moveTo(const geometry_msgs::msg::Pose& pose, const std::string& what)
{
  stage(what);
  arm_.setStartStateToCurrentState();
  arm_.setPoseTarget(pose, kTipLink);
  planAndExecute(arm_, what);
}

void SkillsServer::gripperTo(const std::string& state)
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

void SkillsServer::planAndExecute(MoveGroupInterface& group, const std::string& what)
{
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    throw Failure("planning failed: " + what);
  throwIfCanceled(what);  // the cancel may have arrived while planning
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    throw Failure("execution failed: " + what);
}

void SkillsServer::straightTo(const geometry_msgs::msg::Pose& pose, const std::string& what)
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

void SkillsServer::checkJointJumps(const trajectory_msgs::msg::JointTrajectory& traj, const std::string& what) const
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

void SkillsServer::logToolError(const geometry_msgs::msg::Pose& target)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let the arm settle
  const auto actual = arm_.getCurrentPose(kTipLink).pose.position;
  RCLCPP_INFO(log_, "Tool at (%.4f, %.4f, %.4f), target (%.4f, %.4f, %.4f): error %.1f mm", actual.x, actual.y,
              actual.z, target.position.x, target.position.y, target.position.z,
              1000.0 * std::hypot(actual.x - target.position.x, actual.y - target.position.y,
                                  actual.z - target.position.z));
}
}  // namespace so101_skills
