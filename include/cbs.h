#pragma once

#include "cbs_tree.h"
#include "utils.h"
#include "solution.h"
// #include "planner.h"

// allocates on heap and returns successors
vector<CBSNode*> generateSuccessorsCBS(
    CBSNode* node,
    ManipulatorPlanner* planner,
    const MultiState& startPos,
    size_t constraintInterval
);
// evaluate node and updates internal fields such as
// solution, sumG, conflictsNumber etc
// return true if need to pay reward, instead of penalty
bool evaluateNode(
    CBSNode* node,
    ManipulatorPlanner* planner,
    const MultiState &startPos,
    const MultiState &goalPos,
    double weight, double time
);

MultiSolution CBS(
    size_t dof, size_t arms,
    ManipulatorPlanner* planner,
    const MultiState& startPos,
    const MultiState& goalPos,
    double weight = 1.0,
    double timeLimit = 1.0,
    size_t constraintInterval = 1
);
