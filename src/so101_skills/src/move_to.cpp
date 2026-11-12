#include "skills_server.hpp"

namespace so101_skills
{
void SkillsServer::moveToNamed(const MoveTo::Goal& goal, MoveTo::Result& result)
{
  stage(goal.named_target);
  arm_.setStartStateToCurrentState();
  if (!arm_.setNamedTarget(goal.named_target))
    throw Failure("no named state '" + goal.named_target + "' for the arm group (SRDF)");
  planAndExecute(arm_, goal.named_target);
  result.message = "at '" + goal.named_target + "'";
}
}  // namespace so101_skills
