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

Solution Solution::reversed() const
{
    Solution reversed = Solution(_primitiveActions, _zeroAction);
    reversed.stats = stats;
    reversed._nextActionId = 0;
    for (int i = _solveActions.size() - 1; i >= 0; --i)
    {
        reversed.addAction(_primitiveActions.size() - 1 - _solveActions[i]);
    }
    return reversed;
}

void Solution::add(const Solution &solution)
{
    // TODO implement
    // if (_primitiveActions != solution._primitiveActions)
    // {
    //     printf("Solution::add: different primitive actions\n");
    //     return;
    // }
    for (int i = 0; i < solution._solveActions.size(); ++i)
    {
        addAction(solution._solveActions[i]);
    }
    stats.expansions += solution.stats.expansions;
    stats.consideredEdges += solution.stats.consideredEdges;
    stats.evaluatedEdges += solution.stats.evaluatedEdges;
    stats.pathCost += solution.stats.pathCost;
    stats.pathPotentialCost += stats.pathPotentialCost;
    stats.byteSize = std::max(stats.byteSize, solution.stats.byteSize);
}

size_t Solution::size() const
{
    return _solveActions.size();
}

Action Solution::operator[](size_t i) const
{
    if (i >= size())
    {
        return _zeroAction;
    }
    return _primitiveActions[_solveActions[i]];
}

size_t Solution::byteSize() const
{
    return _zeroAction.byteSize() * (1 + _primitiveActions.size())
    + sizeof(_nextActionId) * (1 + _solveActions.size());
}
