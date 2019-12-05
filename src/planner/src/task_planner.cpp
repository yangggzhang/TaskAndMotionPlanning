#include "planner/task_planner.h"

namespace tamp {
namespace planner {

TaskPlanner::TaskPlanner(const std::string& description_file_path){
	env = create_env(const_cast<char*>(description_file_path.c_str()));
    if (print_status)
    {
        cout << *env;
    }
}
std::vector<GroundedAction> TaskPlanner::run(Heuristic heuristicOption=NoHeuristic)
{
    // this is where you insert your planner
    std::vector<GroundedAction> actions;
    clock_t start_t,end_t;
    double total_t;
    start_t = clock();
    // initial env
    GroundedConditionSet current_conditions = env->get_initial_conditions();
    GroundedConditionSet goal_conditions = env->get_goal_conditions();
    list<GroundedAction> all_possible_actions = env->get_all_grounded_actions();
    // // generate intial plan
    actions = aStar(current_conditions,
                    goal_conditions,
                    all_possible_actions,
                    heuristicOption);
    std::cout << "\nPlan: " << std::endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    TmpOutput task_motion_output;
    if (actions.empty())
    {
        task_motion_output.plan_status = FAILED;
    }
    else
    {
        task_motion_output = interface(actions); // SUCCESS, FAILED, BLOCKED
    }

    while(task_motion_output.plan_status == REPLAN)
    {
        // update_state
        // apply successful actions
        for (int i=0; i<task_motion_output.failed_step_index; i++)
        {
            current_conditions = applyAction(actions[i], current_conditions);
        }
        // remove not block
        std::list<std::string> failed_objects = actions[task_motion_output.failed_step_index].get_arg_values();
        for (std::string obstacle : task_motion_output.obstacles)
        {
            if (failed_objects.size()>1)
            {
                std::cout<<"failed action arguments size > 1"<<std::endl;
                throw;
            }
            GroundedCondition condition_to_remove = GroundedCondition("NotBlock", {obstacle, *(failed_objects.begin())});
            cout<<condition_to_remove.toString()<<endl;
            current_conditions.erase(condition_to_remove);
        }
        // replan
        actions = aStar(current_conditions,
                        goal_conditions,
                        all_possible_actions,
                        heuristicOption);
        if (actions.empty())
        {
            task_motion_output.plan_status = FAILED;
            break;
        }
        task_motion_output = interface(actions);
        std::cout << "\nPlan: " << std::endl;
        for (GroundedAction gac : actions)
        {
            cout << gac << endl;
        }

    }

    if ((task_motion_output.plan_status == SUCCESS))
    {
        std::cout<<"FINAL SUCCESS"<<std::endl;
    }
    else
    {
        std:cout<<"failed to find feasible goal"<<std::endl;
    }
    
    end_t = clock();
    total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
    cout<<"Finish A*! Time: "<<total_t;
    return actions;
}

}  // namespace planner
}  // namespace tamp