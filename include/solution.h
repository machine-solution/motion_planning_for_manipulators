#pragma once

#include "joint_state.h"
#include "utils.h"

enum PathVerdict
{
    PATH_NOT_FOUND,
    PATH_FOUND,
    PATH_NOT_EXISTS,
};

struct Stats
{
    size_t expansions = 0;
    CostType pathCost = 0;
    CostType pathPotentialCost = 0;
    size_t byteSize = 0;
    size_t preprocByteSize = 0;
    int pathVerdict = PATH_NOT_FOUND;
    size_t evaluatedEdges = 0;
    size_t consideredEdges = 0;

    double runtime = 0.0;
    double preprocRuntime = 0.0;
};

class Solution
{
public:
    Solution();
    Solution(const vector<Action>& primitiveActions, const Action& zeroAction);

    Action& nextAction();
    int thisActionId() const;
    void addAction(size_t stepId);

    bool goalAchieved() const;

    // save stats, not save profiling
    Solution reversed() const;

    // merge some stats
    void add(const Solution& solution);

    size_t size() const;
    Action operator[](size_t i) const;

    void reset();

    Stats stats;

    vector<ProfileInfo> plannerProfile;
    vector<ProfileInfo> searchTreeProfile;

    size_t byteSize() const;

private:
    vector<Action> _primitiveActions; // from Planner
    Action _zeroAction;
    vector<size_t> _solveActions; // vector id-s of primitiveActions
    size_t _nextActionId;
};


class StateChain
{
public:
    StateChain(JointState startState, Solution solution);

    JointState operator[](int i) const;
    JointState back() const;

    size_t size() const;
private:
    JointState _startState;
    vector<JointState> _states;
};


class MultiSolution
{
public:
    MultiSolution();
    MultiSolution(const vector<Action>& primitiveActions, const Action& zeroAction, size_t dof, size_t arms);

    size_t arms() const;
    Solution& operator[](size_t i);

    MultiAction nextAction();

    void reset();

    size_t countActions() const;

    bool goalAchieved() const;

    Stats stats;
private:
    vector<Action> _primitiveActions; // from Planner
    Action _zeroAction;

    vector<Solution> _solutions; // solutions for every arm
    size_t _dof;
    size_t _arms;
    size_t _nextActionId = 0;
};
