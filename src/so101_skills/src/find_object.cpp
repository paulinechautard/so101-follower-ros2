#include <chrono>
#include <cmath>
#include <future>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "skills_server.hpp"

namespace so101_skills
{
void SkillsServer::findObject(const FindObject::Goal& goal, FindObject::Result& result)
{
  RCLCPP_INFO(log_, "Goal: find '%s'", goal.object_id.c_str());
  if (held_ && held_->id == goal.object_id)
  {
    RCLCPP_WARN(log_, "'%s' was held since a pick: forgetting it", held_->id.c_str());
    held_.reset();
  }
  if (use_camera_)
  {
    std::optional<std::vector<double>> watch;
    if (!goal.look_at.header.frame_id.empty())
    {
      geometry_msgs::msg::PoseStamped look_at;
      look_at.header = goal.look_at.header;
      look_at.pose.position = goal.look_at.point;
      watch = watchJoints(toPlanningFrame(look_at, "look_at").translation());
      if (!watch)
        RCLCPP_WARN(log_, "No look pose aims the camera at look_at: sweeping the views only");
    }
    lookAndDetect(goal.object_id, watch);
  }
  const Box object = sceneObject(goal.object_id);
  result.object_pose = stamped(object.pose);
  const auto& p = object.pose.translation();
  result.message = "'" + goal.object_id + "' at (" + std::to_string(p.x()) + ", " + std::to_string(p.y()) + ", " +
                   std::to_string(p.z()) + ")";
}

std::optional<std::vector<double>> SkillsServer::watchJoints(const Eigen::Vector3d& target)
{
  moveit::core::RobotState state(arm_.getRobotModel());
  state.setToDefaultValues();
  const auto* group = state.getJointModelGroup(kArmGroup);
  // Azimuth of the point the camera's optical axis hits at target's height
  auto azimuth = [&](double pan) -> std::optional<double> {
    auto joints = look_joints_;
    joints[pan_index_] = pan;
    state.setJointGroupPositions(group, joints);
    state.update();
    const Eigen::Isometry3d& camera = state.getGlobalLinkTransform(kCameraFrame);
    const Eigen::Vector3d axis = camera.linear().col(2);
    if (axis.z() > -0.1)
      return std::nullopt;  // not looking down
    const Eigen::Vector3d hit = camera.translation() + axis * ((target.z() - camera.translation().z()) / axis.z());
    return std::atan2(hit.y(), hit.x());
  };
  const double wanted = std::atan2(target.y(), target.x());
  double pan = look_joints_[pan_index_];
  for (int i = 0; i < 4; ++i)  // Newton's method: the view turns about as much as the pan
  {
    const auto a = azimuth(pan), b = azimuth(pan + 0.05);
    if (!a || !b || std::abs(*b - *a) < 1e-3)
      return std::nullopt;
    pan += std::remainder(wanted - *a, 2.0 * M_PI) * 0.05 / std::remainder(*b - *a, 2.0 * M_PI);
  }
  auto joints = look_joints_;
  joints[pan_index_] = pan;
  state.setJointGroupPositions(group, joints);
  if (!state.satisfiesBounds(group))
    return std::nullopt;
  return joints;
}

// Look for the object from watch (when given) then from look_joints and the same pose
// turned by each of look_pan_offsets until object_detector finds it
void SkillsServer::lookAndDetect(const std::string& id, const std::optional<std::vector<double>>& watch)
{
  std::vector<std::pair<std::string, std::vector<double>>> views;
  if (watch)
    views.emplace_back("watch", *watch);
  for (size_t i = 0; i < look_pan_offsets_.size(); ++i)
  {
    auto joints = look_joints_;
    joints[pan_index_] += look_pan_offsets_[i];
    views.emplace_back("view " + std::to_string(i + 1) + "/" + std::to_string(look_pan_offsets_.size()), joints);
  }
  const std::string service = detect_client_->get_service_name();
  std::string last_error;
  for (const auto& [view, joints] : views)
  {
    stage("look (" + view + ")");
    arm_.setStartStateToCurrentState();
    if (!arm_.setJointValueTarget(joints))
    {
      RCLCPP_WARN(log_, "%s: shoulder_pan %.2f is out of its limits, skipped", view.c_str(), joints[pan_index_]);
      continue;
    }
    planAndExecute(arm_, "look, " + view);

    stage("detect (" + view + ")");
    if (!detect_client_->wait_for_service(std::chrono::seconds(2)))
      throw Failure(service + " not available (is object_detector running?)");
    auto request = std::make_shared<DetectObjects::Request>();
    request->object_ids = { id };
    auto future = detect_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready)
      throw Failure(service + " timed out");
    const auto response = future.get();
    if (response->success)
    {
      RCLCPP_INFO(log_, "Camera (%s): %s", view.c_str(), response->message.c_str());
      return;
    }
    last_error = response->message;
    RCLCPP_INFO(log_, "Camera (%s): %s", view.c_str(), last_error.c_str());
  }
  throw Failure("camera: not found from any view (last: " + last_error + ")");
}
}  // namespace so101_skills
