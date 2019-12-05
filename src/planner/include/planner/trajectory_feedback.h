#pragma once

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlanningScene.h>
#include <string>
#include <vector>
#include "planning_scene.h"

namespace tamp {
namespace feedback {

const std::string kPandaStr = "panda";
const int kMaxContacts = 1000;
const int kMotionPlanStage = 5;
const int kOpenStage = 1;
const int kCloseStage = 3;
const int kArmJointDOF = 7;
const int kHandJointDOF = 2;
const std::string kArmGroup = "panda_arm";
const std::string kHandGroup = "hand";

class TrajectoryFeedback {
 public:
  TrajectoryFeedback() = delete;

  static std::unique_ptr<TrajectoryFeedback> MakeFromShared(
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  bool GetCollisionFeedback(const std::vector<std::string>& scene_objects,
                            const std::string& pickup_object,
                            moveit_msgs::PickupResultConstPtr& plan_result,
                            std::vector<std::string>& collided_objects);

 private:
  TrajectoryFeedback(
      std::shared_ptr<scene::PlanningScene> planning_scene_interface);

  bool HasCollision(
      const std::string& pickup_object,
      const collision_detection::CollisionResult& collision_result,
      std::vector<std::string>& collided_objects_vec);

  bool GetTrajectoryFeedback(planning_scene::PlanningScenePtr& scene,
                             robot_state::RobotState& current_state,
                             const robot_model::JointModelGroup* arm,
                             const std::vector<double>& arm_joints,
                             const robot_model::JointModelGroup* hand,
                             const std::vector<double>& hand_joints,
                             const std::string& pickup_object,
                             std::vector<std::string>& collided_objects);

  std::shared_ptr<scene::PlanningScene> planning_scene_interface_;
};
}  // namespace feedback
}  // namespace tamp