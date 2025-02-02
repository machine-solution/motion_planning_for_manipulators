#include "solution.h"
#include "utils.h"

#include <iostream>

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

int Solution::thisActionId() const
{
    if (_nextActionId >= _solveActions.size())
    {
        return -1;
    }
    return _solveActions[_nextActionId];
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

MultiSolution::MultiSolution()
{
}

MultiSolution::MultiSolution(const vector<Action> &primitiveActions, const Action &zeroAction, size_t dof, size_t arms)
{
    _dof = dof;
    _arms = arms;
    _primitiveActions = primitiveActions;
    _zeroAction = zeroAction;
    _solutions = std::vector<Solution>(_arms, Solution(primitiveActions, zeroAction));
}

size_t MultiSolution::arms() const
{
    return _arms;
}

Solution& MultiSolution::operator[](size_t i)
{
    return _solutions.at(i);
}

MultiAction MultiSolution::nextAction()
{
    vector<Action> actions(_arms, Action(_dof));
    for (size_t a = 0; a < _arms; ++a)
    {
        actions[a] = _solutions[a].nextAction();
    }
    return MultiAction(actions);
}

size_t MultiSolution::countActions() const
{
    size_t count = 0;
    for (size_t i = 0; i < _arms; ++i)
    {
        count += _solutions[i].size();
    }
    return count;
}

bool MultiSolution::goalAchieved() const
{
    for (size_t a = 0; a < _arms; ++a)
    {
        if (!_solutions[a].goalAchieved())
        {
            return false;
        }
    }
    return true;
}

StateChain::StateChain(JointState startState, Solution solution)
{
    _startState = startState;
    while (!solution.goalAchieved())
    {
        startState.apply(solution.nextAction());
        _states.push_back(startState);
    }
}

JointState StateChain::operator[](int i) const
{
    if (i < 0)
    {
        return _startState;
    }
    if (i >= _states.size())
    {
        return back();
    }
    return _states.at(i);
}

JointState StateChain::back() const
{
    if (_states.size() > 0)
    {
        return _states.back();
    }
    else
    {
        return _startState;
    }
}

size_t StateChain::size() const
{
    return _states.size();
}
