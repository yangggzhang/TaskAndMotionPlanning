#include "planner/task_planner_util.h"

namespace tamp {
namespace planner {

GroundedCondition get_ground_condition(Condition condition, unordered_map<string, string> conditionGroundArgMap)
{
    list<string> groundedArgs;
    list<string> conditionArgList = condition.get_args();
    for (string conditionArg:conditionArgList)
    {
        if (conditionGroundArgMap.count(conditionArg)>0)
        {
            groundedArgs.push_back(conditionGroundArgMap[conditionArg]);
        }
        else
        {
            groundedArgs.push_back(conditionArg);// use condition args default
        }
    }
    return GroundedCondition(condition.get_predicate(), groundedArgs, condition.get_truth());
}

GroundedConditionSet get_ground_condition_set(ConditionSet conditions, unordered_map<string, string> conditionGroundArgMap)
{
    GroundedConditionSet ret;
    for (auto condition:conditions)
    {
        ret.insert(get_ground_condition(condition, conditionGroundArgMap));
    }
    return ret;
}

bool compare(int a, int b){
    if(a > b){
        return true;
    }else{
        return false;
    }
}

void printList(list<string> symbols)
{
    for (auto str:symbols)
    {
        cout<<str<<" ";
    }
    cout<<endl;
}

void fullPermutation(list<list<string>>& result, vector<int>& vecInt, vector<string>& symbols, int size)
{
    vector<string> combo;
    vector<int> toPermutate;
    int id = 0;
    for(int i = 0; i < vecInt.size(); ++i){
        if(vecInt[i] == 1)
        {
            combo.push_back(symbols[i]);
            toPermutate.push_back(id);
            id++;
        }
    }
    do 
    {
        list<string> comboList;
        for (int i = 0; i < toPermutate.size(); i++)
        {
            comboList.push_back(combo[toPermutate[i]]);
        }
        result.push_back(comboList);
    } while (next_permutation(toPermutate.begin(),toPermutate.end()));
}

void combination(list<list<string>>& result, vector<string> symbols, int select_size){
    
    int total_size = symbols.size();
    //initial first combination like:1,1,0,0,0
    vector<int> vecInt(total_size,0);
    for(int i = 0; i < select_size; ++i){
        vecInt[i] = 1;
    }
 
    fullPermutation(result, vecInt, symbols, select_size);
 
    for(int i = 0; i < total_size - 1; ++i){
        if(vecInt[i] == 1 && vecInt[i+1] == 0){
            //1. first exchange 1 and 0 to 0 1
            swap(vecInt[i], vecInt[i+1]);
 
            //2.move all 1 before vecInt[i] to left
            sort(vecInt.begin(),vecInt.begin() + i, compare);
 
            //after step 1 and 2, a new combination is exist
            fullPermutation(result, vecInt, symbols, select_size);
 
            //try do step 1 and 2 from front
            i = -1;
        }
    }
}

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

bool isValidAction(const GroundedAction& action, const GroundedConditionSet& currentConditionSet)
{
    GroundedConditionSet preconditions = action.get_preconditions();
    for(GroundedCondition precondition:preconditions){
        if(!precondition.get_truth())
        {
            throw invalid_argument( "Cannot get precondition value!" );
        }
        if(currentConditionSet.count(precondition)==0) return false;
    }
    return true;
}

bool reachGoal(const GroundedConditionSet& currentConditionSet, const GroundedConditionSet& goalConditionSet)
{
    for(GroundedCondition goalCondition:goalConditionSet){
        if(!goalCondition.get_truth())
        {
            throw invalid_argument( "Cannot get goalCondition value!" );
        }
        if(currentConditionSet.count(goalCondition)==0) return false;
    }
    return true;
}

void print(const GroundedConditionSet& conditions){
    for (auto it = conditions.begin(); it != conditions.end(); ++it)
        cout<<(*it);
    cout<<endl<<endl;
    return;
}

GroundedConditionSet applyAction(GroundedAction& action, GroundedConditionSet& currentConditions, bool deletion)
{
    for (auto effect:action.get_effects())
    {
        if (!effect.get_truth() && deletion)
        {
            effect.set_truth(true);
            auto it = currentConditions.find(effect);
            if (it != currentConditions.end())
            {
                currentConditions.erase(it);
            }
            else
            {
                cout<<"cannot find "<<effect.toString();
                // throw invalid_argument("current condition cannot find effect");
            }

        }
        else if (effect.get_truth())
        {
            currentConditions.insert(effect);
        }
        else
        {
            cout<<"no deletion"<<endl;
        }
    }
    return currentConditions;
}

}
}