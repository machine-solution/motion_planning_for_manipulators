#include "solution.h"
#include "utils.h"

Solution::Solution()
{
    _nextActionId = 0;
    _solveActions.clear();
}
Solution::Solution(const vector<Action>& primitiveActions, const Action& zeroAction) : Solution()
{
    _primitiveActions = primitiveActions;
    _zeroAction = zeroAction;
}

Action& Solution::nextAction()
{
    if (_nextActionId >= _solveActions.size())
    {
        return _zeroAction;
    }
    return _primitiveActions[_solveActions[_nextActionId++]];
}
void Solution::addAction(size_t stepId)
{
    _solveActions.push_back(stepId);
}

bool Solution::goalAchieved() const
{
    return _nextActionId >= _solveActions.size();
}
