#pragma once

#include "joint_state.h"
#include "utils.h"

enum PathVerdict
{
    PATH_FOUND,
    PATH_NOT_FOUND,
    PATH_NOT_EXISTS,
};

struct Stats
{
    size_t expansions = 0;
    CostType pathCost = 0;
    CostType pathPotentialCost = 0;
    size_t byteSize = 0;
    int pathVerdict = PATH_NOT_FOUND;
    size_t evaluatedEdges = 0;
    size_t consideredEdges = 0;

    double runtime = 0.0;
};

class Solution
{
public:
    Solution();
    Solution(const vector<Action>& primitiveActions, const Action& zeroAction);

    Action& nextAction();
    void addAction(size_t stepId);

    bool goalAchieved() const;

    Solution reversed() const;

    void add(const Solution& solution);

    size_t size() const;
    Action operator[](size_t i) const;

    Stats stats;

    vector<ProfileInfo> plannerProfile;
    vector<ProfileInfo> searchTreeProfile;

private:
    vector<Action> _primitiveActions; // from Planner
    Action _zeroAction;
    vector<size_t> _solveActions; // vector id-s of primitiveActions
    size_t _nextActionId;
};
