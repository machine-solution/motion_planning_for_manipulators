#include "solution.h"

Solution::Solution()
{
    _nextStepId = 0;
    _solveSteps.clear();
}
Solution::Solution(const vector<JointState>& primitiveSteps, const JointState& zeroStep) : Solution()
{
    _primitiveSteps = primitiveSteps;
    _zeroStep = zeroStep;
}

JointState& Solution::nextStep()
{
    if (_nextStepId >= _solveSteps.size())
    {
        return _zeroStep;
    }
    return _primitiveSteps[_solveSteps[_nextStepId++]];
}
void Solution::addStep(size_t stepId)
{
    _solveSteps.push_back(stepId);
}

bool Solution::goalAchieved() const
{
    return _nextStepId >= _solveSteps.size();
}
