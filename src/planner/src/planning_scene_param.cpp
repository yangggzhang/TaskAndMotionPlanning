#include "planner/planning_scene_param.h"
#include "planner/planning_utils.h"

namespace tamp {
namespace scene {
PlanningSceneParam::PlanningSceneParam(){};

bool PlanningSceneParam::ParseFromRosParam(const ros::NodeHandle &ph) {
  collision_objects_.clear();
  XmlRpc::XmlRpcValue object_param_list;
  ph.getParam("collision_objects", object_param_list);
  for (int32_t i = 0; i < object_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue object_param = object_param_list[i];
    moveit_msgs::CollisionObject collision_object;
    if (!utils::GetParam(object_param, "id", collision_object.id)) {
      return false;
    }

    if (!utils::GetParam(object_param, "frame_id",
                         collision_object.header.frame_id)) {
      return false;
    }

    // BOX 1
    // SPHERE 2
    // CYLINDER 3
    // CONE 4
    int shape_type;
    if (!utils::GetParam(object_param, "shape_type", shape_type)) {
      return false;
    }
    collision_object.primitives.resize(1);
    switch (shape_type) {
      case 1: {
        const int primitive_dimension = 3;
        collision_object.primitives[0].type =
            collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(primitive_dimension);
        std::vector<double> box_dimensions;
        if (!utils::GetParam(object_param, "dimension", box_dimensions)) {
          return false;
        }
        if (box_dimensions.size() != primitive_dimension) {
          return false;
        }
        for (size_t j = 0; j < primitive_dimension; ++j) {
          collision_object.primitives[0].dimensions[j] = box_dimensions[j];
        }
        break;
      }
      case 2: {
        const int primitive_dimension = 1;
        collision_object.primitives[0].type =
            collision_object.primitives[0].SPHERE;
        collision_object.primitives[0].dimensions.resize(primitive_dimension);
        std::vector<double> sphere_dimensions;
        if (!utils::GetParam(object_param, "dimension", sphere_dimensions)) {
          return false;
        }
        if (sphere_dimensions.size() != primitive_dimension) {
          return false;
        }
        for (size_t j = 0; j < primitive_dimension; ++j) {
          collision_object.primitives[0].dimensions[j] = sphere_dimensions[j];
        }
        break;
      }
      case 3: {
        const int primitive_dimension = 2;
        collision_object.primitives[0].type =
            collision_object.primitives[0].CYLINDER;
        collision_object.primitives[0].dimensions.resize(primitive_dimension);
        std::vector<double> cylinder_dimensions;
        if (!utils::GetParam(object_param, "dimension", cylinder_dimensions)) {
          return false;
        }
        if (cylinder_dimensions.size() != primitive_dimension) {
          return false;
        }
        for (size_t j = 0; j < primitive_dimension; ++j) {
          collision_object.primitives[0].dimensions[j] = cylinder_dimensions[j];
        }
        break;
      }
      case 4: {
        const int primitive_dimension = 2;
        collision_object.primitives[0].type =
            collision_object.primitives[0].CONE;
        collision_object.primitives[0].dimensions.resize(primitive_dimension);
        std::vector<double> cone_dimensions;
        if (!utils::GetParam(object_param, "dimension", cone_dimensions)) {
          return false;
        }
        if (cone_dimensions.size() != primitive_dimension) {
          return false;
        }
        for (size_t j = 0; j < primitive_dimension; ++j) {
          collision_object.primitives[0].dimensions[j] = cone_dimensions[j];
        }
        break;
      }
      default: {
        return false;
        break;
      }
    }

    std::vector<double> primitive_poses;
    if (!utils::GetParam(object_param, "primitive_poses", primitive_poses)) {
      return false;
    }
    if (primitive_poses.size() != 3) {
      return false;
    }
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = primitive_poses[0];
    collision_object.primitive_poses[0].position.y = primitive_poses[1];
    collision_object.primitive_poses[0].position.z = primitive_poses[2];

    collision_object.operation = collision_object.ADD;

    collision_objects_.push_back(collision_object);
  }
  return true;
}

std::vector<moveit_msgs::CollisionObject>
PlanningSceneParam::GetCollisionObjects() {
  return collision_objects_;
}

}  // namespace scene
}  // namespace tamp