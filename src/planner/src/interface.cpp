
enum class PlannerStatus {SUCCESS, FAILED, BLOCKED};

// unordered_set<string> generate_partial_scene(vector<GroundedAction>& cur_task_plan){
//     unordered_set<string> partial_scene;
//     for(auto ga : cur_task_plan){
//         partial_scene.insert(.....);
//     }
// }

void interface(vector<GroundedAction>& cur_task_plan, MotionPlan& plan, PlannerStatus& plan_status, vector<string>& obstacles){
    GroundedAction action = cur_task_plan[0];
    // plan motion with whole scene
    PlannerStatus mp_status = motion_planner(action, scene, mp_plan);
    if(mp_status == PlannerStatus::SUCCESS){
        plan = mp_plan;
        plan_status = PlannerStatus::SUCCESS;
        return;
    }
    else{
        //plan with partial scene
        unordered_set<string> partial_scene = generate_partial_scene(cur_task_plan);
        mp_status = motion_planner(action, partial_scene, mp_plan);
        if(mp_status == PlannerStatus::SUCCESS){
            //use collision checker to find which obj blocks the plan
            obstacles = get_obstacles(mp_plan);
            plan_status = PlannerStatus::BLOCKED;
            return;
        }
        else{
            plan_status = PlannerStatus::FAILED;
            return;
        }
    }
}