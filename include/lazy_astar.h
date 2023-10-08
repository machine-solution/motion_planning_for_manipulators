#pragma once

#include "astar.h"

namespace astar
{

vector<SearchNode*> lazyGenerateSuccessors(
    SearchNode* node,
    IAstarChecker& checker,
    double weight
);

Solution lazyAstar(
    const JointState& startPos,
    IAstarChecker& checker,
    double weight = 1.0,
    double timeLimit = 1.0
);

} // namespace astar
