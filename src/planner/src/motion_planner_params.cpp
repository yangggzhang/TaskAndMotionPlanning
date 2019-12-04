#include "planner/motion_planner_params.h"
#include "planner/planning_utils.h"

namespace tamp {
namespace planner {

bool MotionPlannerParams::LoadFromRosParams(const ros::NodeHandle& ph) {
  if (!ph.getParam("move_group", move_group)) {
    ROS_ERROR_STREAM("Missing move group!");
    return false;
  }

  if (!ph.getParam("world_frame", move_group)) {
    ROS_ERROR_STREAM("Missing world frame!");
    return false;
  }

  if (!ph.getParam("planner_name", planner_name)) {
    ROS_ERROR_STREAM("Missing planner name!");
    return false;
  }

  if (!ph.getParam("max_planning_time_sec", max_planning_time_sec)) {
    ROS_ERROR_STREAM("Missing max planning time sec!");
    return false;
  }

  if (!ph.getParam("plan_only", plan_only)) {
    ROS_ERROR_STREAM("Missing plan only!");
    return false;
  }

  if (!ph.getParam("num_planning_attempts", num_planning_attempts)) {
    ROS_ERROR_STREAM("Missing num planning attempts!");
    return false;
  }

  grasps.clear();
  XmlRpc::XmlRpcValue grasps_param_list;
  ph.getParam("grasps", grasps_param_list);
  for (int32_t i = 0; i < grasps_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue grasp_param = grasps_param_list[i];
    MoveItGrasp grasp_candidate;
    if (!utils::GetParam(grasp_param, "pose_orientation",
                         grasp_candidate.pose_orientation)) {
      return false;
    } else {
      if (grasp_candidate.pose_orientation.size() != 3) {
        return false;
      }
    }

    if (!utils::GetParam(grasp_param, "pose_position",
                         grasp_candidate.pose_position)) {
      return false;
    } else {
      if (grasp_candidate.pose_position.size() != 3) {
        return false;
      }
    }

    if (!utils::GetParam(grasp_param, "approach_direction",
                         grasp_candidate.approach_direction)) {
      return false;
    } else {
      if (grasp_candidate.approach_direction.size() != 3) {
        return false;
      }
    }

    if (!utils::GetParam(grasp_param, "retreat_direction",
                         grasp_candidate.retreat_direction)) {
      return false;
    } else {
      if (grasp_candidate.retreat_direction.size() != 3) {
        return false;
      }
    }

    if (!utils::GetParam(grasp_param, "approach_min_distance",
                         grasp_candidate.approach_min_distance)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "approach_desired_distance",
                         grasp_candidate.approach_desired_distance)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "retreat_min_distance",
                         grasp_candidate.retreat_min_distance)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "retreat_desired_distance",
                         grasp_candidate.retreat_desired_distance)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "gripper_joint_names",
                         grasp_candidate.gripper_joint_names)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "gripper_open_joint_positions",
                         grasp_candidate.gripper_open_joint_positions)) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "gripper_closed_joint_positions",
                         grasp_candidate.gripper_closed_joint_positions)) {
      return false;
    }

    if (grasp_candidate.gripper_joint_names.size() !=
            grasp_candidate.gripper_open_joint_positions.size() ||
        grasp_candidate.gripper_joint_names.size() !=
            grasp_candidate.gripper_closed_joint_positions.size()) {
      return false;
    }

    if (!utils::GetParam(grasp_param, "gripper_time_from_start",
                         grasp_candidate.gripper_time_from_start)) {
      return false;
    }

    grasps.push_back(grasp_candidate);
  }
  return true;
}

}  // namespace planner
}  // namespace tamp