#include "planner/task_planner.h"

namespace tamp {
namespace planner {

std::unique_ptr<TaskPlanner> TaskPlanner::MakeFromRosParam(const ros::NodeHandle &ph) {

    std::string df;
    if (!ph.getParam("description_file", df)) {
        ROS_ERROR_STREAM("Missing description file path!");
        return nullptr;
    }
    return std::unique_ptr<TaskPlanner>(new TaskPlanner(df));
}

TaskPlanner::TaskPlanner(const std::string& description_file_path) {

}


}  // namespace planner
}  // namespace tamp