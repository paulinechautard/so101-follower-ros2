#include <cmath>

#include "skills_server.hpp"

namespace so101_skills
{
void SkillsServer::place(const Place::Goal& goal, Place::Result& result)
{
  if (!held_)
    throw Failure("not holding anything: send a pick goal first");
  const Grasp grasp = *held_;
  RCLCPP_INFO(log_, "Goal: place '%s'", grasp.id.c_str());
  const Eigen::Isometry3d place = toPlanningFrame(goal.place_pose, "place_pose");
  // The object turns with the gripper
  const double place_yaw = std::atan2(place.linear()(1, 0), place.linear()(0, 0)) + grasp.quarters * M_PI_2;
  // Put down 2 mm high rather than pushing into the support
  const double release_z = place.translation().z() + grasp.height / 2.0 - grasp_depth_ + 0.002;
  const Eigen::Vector3d drop = place.translation();
  auto at_place = [&](double z) { return downPose(drop.x(), drop.y(), z, place_yaw, grasp.width); };

  moveTo(at_place(release_z + approach_), "above the place spot");
  straightTo(at_place(release_z), "descend to place");
  gripperTo("open");
  detach(grasp.id);
  held_.reset();
  straightTo(at_place(release_z + approach_), "retreat");
  result.message = "placed '" + grasp.id + "'";
}
}  // namespace so101_skills
