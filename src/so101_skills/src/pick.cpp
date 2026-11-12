#include <cmath>

#include "skills_server.hpp"

namespace so101_skills
{
void SkillsServer::pick(const Pick::Goal& goal, Pick::Result& result)
{
  RCLCPP_INFO(log_, "Goal: pick '%s'", goal.object_id.c_str());
  if (held_)
  {
    // If the gripper is holding anything (previous place that failed): drop it
    RCLCPP_WARN(log_, "'%s' was still held: letting go of it", held_->id.c_str());
    held_.reset();
  }
  const Box object = sceneObject(goal.object_id);
  result.object_pose = stamped(object.pose);

  const Eigen::Matrix3d& rotation = object.pose.linear();
  if (rotation(2, 2) < 0.99)
    throw Failure("'" + goal.object_id + "' is not upright: can only grasp from above");
  const double object_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Vector3d pick = object.pose.translation();
  const double facing = std::atan2(pick.y(), pick.x()) + yaw_offset_;
  const double turn = std::remainder(facing - object_yaw, 2.0 * M_PI);
  double quarters;
  if (std::abs(object.size.x() - object.size.y()) < 1e-4)
    quarters = std::round(turn / M_PI_2);  // square base: any side
  else
  {
    const double parity = object.size.x() < object.size.y() ? 0.0 : 1.0;
    quarters = parity + 2.0 * std::round((turn - parity * M_PI_2) / M_PI);
  }
  const double grasp_yaw = object_yaw + quarters * M_PI_2;
  const double width = std::fmod(std::abs(quarters), 2.0) == 0.0 ? object.size.x() : object.size.y();

  if (grasp_depth_ >= object.size.z())
    throw Failure("grasp_depth must be less than the object's height");
  const double grasp_z = pick.z() + object.size.z() / 2.0 - grasp_depth_;
  auto at_pick = [&](double z) { return downPose(pick.x(), pick.y(), z, grasp_yaw, width); };

  gripperTo("open");
  moveTo(at_pick(grasp_z + approach_), "above the object");
  straightTo(at_pick(grasp_z), "descend to grasp");
  logToolError(at_pick(grasp_z));
  attach(goal.object_id);
  held_ = Grasp{ goal.object_id, quarters, width, object.size.z() };
  gripperTo("closed");
  straightTo(at_pick(grasp_z + approach_), "lift");
  result.message = "holding '" + goal.object_id + "'";
}
}  // namespace so101_skills
