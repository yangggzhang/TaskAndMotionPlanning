#include "planner/task_planner.h"

namespace tamp {
namespace planner {
std::unique_ptr<TaskPlanner> TaskPlanner::MakeFromRosParam(
    const ros::NodeHandle& ph) {
  std::string df;
  if (!ph.getParam("description_file", df)) {
    ROS_ERROR_STREAM("Missing description file path!");
    return nullptr;
  }
  std::unique_ptr<TaskAndMotionPlanner> task_and_motion_planner =
      TaskAndMotionPlanner::Make(ph);
  if (task_and_motion_planner == nullptr) {
    ROS_ERROR_STREAM("Failed to make TaskAndMotionPlanner!");
    return nullptr;
  }
  return std::unique_ptr<TaskPlanner>(
      new TaskPlanner(df, std::move(task_and_motion_planner)));
}

TmpOutput TaskPlanner::interface(std::vector<GroundedAction> actions) {
  // PlannerStatus plan_status;
  // int fail_step_index;
  // std::std::vector<std::string> obstacles;
  TmpOutput test;
  if ((actions[0].toString()) == "Pick(C3)") {
    test.plan_status = PlannerStatus::REPLAN;
    test.fail_step_index = 0;
    test.obstacles = {"C2"};
  } else if ((actions[0].toString()) == "Pick(C2)") {
    test.plan_status = PlannerStatus::REPLAN;
    test.fail_step_index = 0;
    test.obstacles = {"C3"};
  } else {
    test.plan_status = PlannerStatus::SUCCESS;
    test.obstacles = {};
  }
  return test;
}

TaskPlanner::TaskPlanner(
    const std::string& description_file_path,
    std::unique_ptr<TaskAndMotionPlanner> task_and_motion_planner)
    : task_and_motion_planner_(std::move(task_and_motion_planner)) {
  env = create_env(const_cast<char*>(description_file_path.c_str()));
}
std::vector<GroundedAction> TaskPlanner::run(Heuristic heuristicOption) {
  // this is where you insert your planner
  std::vector<GroundedAction> actions;
  clock_t start_t, end_t;
  double total_t;
  start_t = clock();
  // initial env
  GroundedConditionSet current_conditions = env->get_initial_conditions();
  GroundedConditionSet goal_conditions = env->get_goal_conditions();
  list<GroundedAction> all_possible_actions = env->get_all_grounded_actions();
  // // generate intial plan
  actions = aStar(current_conditions, goal_conditions, all_possible_actions,
                  heuristicOption);
  std::cout << "\nPlan: " << std::endl;
  for (GroundedAction gac : actions) {
    cout << gac << endl;
  }

  TmpOutput task_motion_output;
  if (actions.empty()) {
    task_motion_output.plan_status = PlannerStatus::FAILED;
  } else {
    task_motion_output = interface(actions);  // SUCCESS, FAILED, BLOCKED
  }

  while (task_motion_output.plan_status == PlannerStatus::REPLAN) {
    // update_state
    // apply successful actions
    for (int i = 0; i < task_motion_output.fail_step_index; i++) {
      current_conditions = applyAction(actions[i], current_conditions);
    }
    // remove not block
    std::list<std::string> failed_objects =
        actions[task_motion_output.fail_step_index].get_arg_values();
    for (std::string obstacle : task_motion_output.obstacles) {
      if (failed_objects.size() > 1) {
        std::cout << "failed action arguments size > 1" << std::endl;
        throw;
      }
      GroundedCondition condition_to_remove =
          GroundedCondition("NotBlock", {obstacle, *(failed_objects.begin())});
      cout << condition_to_remove.toString() << endl;
      current_conditions.erase(condition_to_remove);
    }
    // replan
    actions = aStar(current_conditions, goal_conditions, all_possible_actions,
                    heuristicOption);
    if (actions.empty()) {
      task_motion_output.plan_status = PlannerStatus::FAILED;
      break;
    }
    task_motion_output = interface(actions);
    std::cout << "\nPlan: " << std::endl;
    for (GroundedAction gac : actions) {
      cout << gac << endl;
    }
  }

  if ((task_motion_output.plan_status == PlannerStatus::SUCCESS)) {
    std::cout << "FINAL SUCCESS" << std::endl;
  } else {
  std:
    cout << "failed to find feasible goal" << std::endl;
  }

  end_t = clock();
  total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
  cout << "Finish Time: " << total_t << std::endl;
  return actions;
}

std::vector<GroundedAction> TaskPlanner::aStar(
    const GroundedConditionSet& startConditionSet,
    const GroundedConditionSet& goalConditionSet,
    const list<GroundedAction>& allPossibleActions,
    const Heuristic& heuristicOption, const bool& deletion, const bool& print) {
  std::vector<GroundedAction> actions;
  priority_queue<Node*, vector<Node*>, NodeValueComparator> open;
  unordered_set<Node*, NodeStateHasher, NodeStateComparator> closed;

  Node* currentNode =
      new Node(startConditionSet, 0, 0, nullptr, (*allPossibleActions.begin()));
  open.push(currentNode);
  bool findGoal = false;
  while (!open.empty()) {
    if (print) {
      cout << " Closed list size:" << closed.size() << endl;
    }
    currentNode = open.top();
    open.pop();
    if (closed.count(currentNode) > 0) {
      continue;
    }
    closed.insert(currentNode);

    // check if reach goal
    GroundedConditionSet currentCond = currentNode->get_current_conditions();
    if (reachGoal(currentCond, goalConditionSet)) {
      // cout<<"Goal Reached!"<<endl;
      findGoal = true;
      break;
    }
    // expand successors
    for (auto action : allPossibleActions) {
      currentCond = currentNode->get_current_conditions();
      if (isValidAction(action, currentCond)) {
        GroundedConditionSet newConditions =
            applyAction(action, currentCond);  // current cond will change here
        int new_g = currentNode->get_g() + 1;
        int new_f = new_g + getHeuristic(heuristicOption, goalConditionSet,
                                         newConditions, allPossibleActions);
        Node* newNode =
            new Node(newConditions, new_g, new_f, currentNode, action);
        if (closed.count(newNode) == 0) {
          open.push(newNode);
        }
      }
    }
  }

  // traceback actions
  if (findGoal) {
    while (currentNode->get_parent_node()) {
      actions.insert(actions.begin(), currentNode->get_action_from());
      currentNode = currentNode->get_parent_node();
    }
  }
  return actions;
}

int TaskPlanner::getHeuristic(const Heuristic& heuristicOption,
                              const GroundedConditionSet& goalSet,
                              const GroundedConditionSet& currentSet,
                              const list<GroundedAction>& allPossibleActions) {
  int h = 0;
  switch (heuristicOption) {
    case NoHeuristic: {
      break;
    }
    case LiteralHeuristic: {
      for (GroundedCondition goalCondition : goalSet) {
        if (currentSet.count(goalCondition) == 0)  // not satisfied state
        {
          h++;
        }
      }
      break;
    }
    case RelaxedHeuristic: {
      auto actions = aStar(currentSet, goalSet, allPossibleActions, NoHeuristic,
                           false, false);
      h += (int)actions.size();
      break;
    }
    default: { break; }
  }
  return h;
}

}  // namespace planner
}  // namespace tamp