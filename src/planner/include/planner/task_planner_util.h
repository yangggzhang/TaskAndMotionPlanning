#pragma once
#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>

namespace tamp {
namespace planner {
#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

using namespace std;
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

enum Heuristic
{
    NoHeuristic       = 0, 
    LiteralHeuristic  = 1, 
    RelaxedHeuristic  = 2
};
static vector<string> HeuristicName{"NoHeuristic", "LiteralHeuristic", "RelaxedHeuristic"};
// //to be delete-------------------------------------
// enum PlannerStatus
// {
//     SUCCESS       = 0, 
//     FAILED        = 1,
//     REPLAN        = 2
// };

// struct TmpOutput
// {
//     PlannerStatus plan_status;
//     int fail_step_index;
//     std::vector<std::string> obstacles;
// };
// //to be delete------------------------------------
class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    void set_truth(bool truth_in)
    {
        this->truth = truth_in;
    }
    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> GroundedConditionSet;

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

typedef unordered_set<Condition, ConditionHasher, ConditionComparator> ConditionSet;

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

typedef unordered_set<Action, ActionHasher, ActionComparator> ActionSet;

GroundedCondition get_ground_condition(Condition condition, unordered_map<string, string> conditionGroundArgMap);
GroundedConditionSet get_ground_condition_set(ConditionSet conditions, unordered_map<string, string> conditionGroundArgMap);

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(Action action, list<string> arg_values)
    {
        this->name = action.get_name();
        list<string> defaultArgs = action.get_args();

        // check arg size
        if (defaultArgs.size()!=arg_values.size())
        {
            throw invalid_argument("Argument size does not match");
        }
        auto it = defaultArgs.begin();
        unordered_map<string, string> argmap;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
            argmap[*it] = ar;
            it++;
        }
        this->gPreconditions = get_ground_condition_set(action.get_preconditions(),argmap);
        this->gEffects = get_ground_condition_set(action.get_effects(),argmap);
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->gPreconditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->gEffects;
    }
};

bool compare(int a, int b);
void printList(list<string> symbols);
void fullPermutation(list<list<string>>& result, vector<int>& vecInt, vector<string>& symbols, int size);
void combination(list<list<string>>& result, vector<string> symbols, int select_size);

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions; // all actions
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions()
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions()
    {
        return this->goal_conditions;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions()
    {
        return this->actions;
    }

    list<GroundedAction> get_all_grounded_actions()
    {
        list<GroundedAction> ret;
        vector<string> symbols(this->symbols.begin(), this->symbols.end());
        for (auto action:(this->actions))
        {
            list<list<string>> actionArgs;
            combination(actionArgs, symbols, action.get_args().size());// Do mAn (1. do mCn, 2. do mAm)
            for (auto arg:actionArgs)
            {
                GroundedAction ground_action(action, arg);
                ret.push_back(ground_action);
            }
        }
        return ret;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

list<string> parse_symbols(string symbols_str);

Env* create_env(char* filename);

bool isValidAction(const GroundedAction& action, const GroundedConditionSet& currentConditionSet);

bool reachGoal(const GroundedConditionSet& currentConditionSet, const GroundedConditionSet& goalConditionSet);

void print(const GroundedConditionSet& conditions);

GroundedConditionSet applyAction(GroundedAction& action, GroundedConditionSet& currentConditions, bool deletion=true);
class Node
{
private:
    GroundedAction previousAction;
    Node* parentNode;
    GroundedConditionSet currentConditionSet;
    int g = -1;
    int f = -1;
public:
    Node(GroundedConditionSet conditionSet, int gValue, int fValue, Node* parent, GroundedAction prevAction):
        previousAction(prevAction),
        parentNode(parent),
        currentConditionSet(conditionSet),
        f(fValue),
        g(gValue){}
    GroundedAction get_action_from()
    {
        return previousAction;
    }
    Node* get_parent_node()
    {
        return parentNode;
    }
    GroundedConditionSet get_current_conditions()
    {
        return currentConditionSet;
    }
    int get_g()
    {
        return g;
    }
    int get_f()
    {
        return f;
    }
    string toString()
    {
        string ret;
        list<string> conditions;
        for (auto condition:currentConditionSet)
        {
            conditions.push_back(condition.toString());
        }
        conditions.sort();
        for (auto str:conditions)
        {
            ret+=str+" ";
        }
        return ret;
    }
    
};

// compare nodes with their values (for A*)
struct NodeValueComparator
{
    bool operator()(Node* lhs, Node* rhs) const
    {
        return lhs->get_f() > rhs->get_f();
    }
};

// compare nodes with their states(check if is the same state)
struct NodeStateComparator
{
    bool operator()(Node* lhs, Node* rhs) const
    {
        return lhs->toString() == rhs->toString();
    }
};

struct NodeStateHasher
{
    size_t operator()(Node* node) const
    {
        return hash<string>{}(node->toString());
    }
};

}
}