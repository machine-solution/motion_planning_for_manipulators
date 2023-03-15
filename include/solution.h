#pragma once

#include "joint_state.h"

struct Stats
{
    size_t expansions = 0;
    double runtime = 0.0;
    int pathCost = 0;
    size_t maxTreeSize = 0;
    bool pathFound = false;
};

class Solution
{
public:
    Solution();
    Solution(const vector<JointState>& primitiveSteps, const JointState& zeroStep);

    JointState& nextStep();
    void addStep(size_t stepId);

    bool goalAchieved() const;

    Stats stats;

private:
    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    vector<size_t> _solveSteps; // vector id-s of primitiveSteps
    size_t _nextStepId;
};
