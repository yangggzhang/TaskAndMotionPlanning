#include "planner/planning_scene.h"
#include "planner/planning_utils.h"

namespace tamp {
namespace scene {
std::unique_ptr<PlanningScene> PlanningScene::MakeFromRosParam(
    const ros::NodeHandle &ph) {
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  if (!LoadCollisionObjects(ph, collision_objects)) {
    ROS_ERROR(
        "Failed to load collision objects information for planning scene!");
    return nullptr;
  } else
    return std::unique_ptr<PlanningScene>(new PlanningScene(collision_objects));
}

PlanningScene::PlanningScene(
    const std::vector<moveit_msgs::CollisionObject> &collision_objects)
    : collision_objects_(collision_objects) {
  scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  scene_->applyCollisionObjects(collision_objects);
}

bool PlanningScene::LoadCollisionObjects(
    const ros::NodeHandle &ph,
    std::vector<moveit_msgs::CollisionObject> &collision_objects) {
  collision_objects.clear();
  XmlRpc::XmlRpcValue object_param_list;
  ph.getParam("collision_objects", object_param_list);

  for (int32_t i = 0; i < object_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue object_param = object_param_list[i];
    moveit_msgs::CollisionObject collision_object;
    std::string id;
    if (!utils::GetParam(object_param, "id", id)) {
      return false;
    }
    collision_object.id = id;

    int shape_type;

    // BOX 1
    // SPHERE 2
    // CYLINDER 3
    // CONE 4
    if (!utils::GetParam(object_param, "shape_type", shape_type)) {
      return false;
    }

    switch (shape_type) {
      case 1: {
        const int primitive_dimension = 3;
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
    if (primitive_poses.size() == 3) {
      return false;
    }
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = primitive_poses[0];
    collision_object.primitive_poses[0].position.y = primitive_poses[1];
    collision_object.primitive_poses[0].position.z = primitive_poses[2];

    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
  }
  return true;
}

bool PlanningScene::reset() {
  scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  if (collision_objects_.empty()) {
    ROS_ERROR("Missing collision objects in the scene!");
    return false;
  }
  scene_->applyCollisionObjects(collision_objects_);

  return true;
}

}  // namespace scene
}  // namespace tamp