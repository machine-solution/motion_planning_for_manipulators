#include "planner.h"
#include "utils.h"
#include "light_mujoco.h"

#include <time.h>

#include <stdio.h>

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveActions();
}

size_t ManipulatorPlanner::dof() const
{
    return _dof;
}

bool ManipulatorPlanner::checkCollision(const JointState& position) const
{
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position.rad(i);
    }
    return mj_light_collision(_model, _data);
}

bool ManipulatorPlanner::checkCollisionAction(const JointState& start, const Action& action) const
{
    startProfiling();
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = start.rad(i);
    }

    int jump = 8;
    for (size_t t = jump; t <= g_unitSize; t += jump)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i] = start.rad(i) + g_worldEps * action[i] * t; // temporary we use global constant here for speed
        }
        if (mj_light_collision(_model, _data))
        {
            stopProfiling();
            return true;
        }
    }
    stopProfiling();
    return false;
}

vector<string> ManipulatorPlanner::configurationSpace() const
{
    vector<string> cSpace(g_units * 2, string(g_units * 2, '.'));
    for (int i = -g_units; i < g_units; ++i)
    {
        for (int j = -g_units; j < g_units; ++j)
        {
            if (checkCollision({i, j}))
                cSpace[g_units - 1 - j][i + g_units] = '@';
        }
    }
    return cSpace;
}

Solution ManipulatorPlanner::planActions(const JointState& startPos, const JointState& goalPos, int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(startPos) || checkCollision(goalPos))
    {
        Solution solution(_primitiveActions, _zeroAction);
        solution.stats.pathVerdict = PATH_NOT_EXISTS; // incorrect aim
        return  solution;
    }
    switch (alg)
    {
    case ALG_LINEAR:
        return linearPlanning(startPos, goalPos);
    case ALG_ASTAR:
        return astarPlanning(startPos, goalPos, w, timeLimit);
    default:
        return Solution(_primitiveActions, _zeroAction);
    }
}

Solution ManipulatorPlanner::planActions(const JointState& startPos, double goalX, double goalY, int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(startPos))
    {
        Solution solution(_primitiveActions, _zeroAction);
        solution.stats.pathVerdict = PATH_NOT_EXISTS; // incorrect aim
        return  solution;
    }
    switch (alg)
    {
    case ALG_ASTAR:
        return astarPlanning(startPos, goalX, goalY, w, timeLimit);
    default:
        return Solution(_primitiveActions, _zeroAction);
    }
}

double ManipulatorPlanner::modelLength() const
{
    static double len = 0;
    if (len == 0)
    {
        for (size_t i = 1; i <= _dof; ++i)
        {
            len += _model->geom_size[i * 3 + 1] * 2;
        }
    }
    return len;
}
double ManipulatorPlanner::maxActionLength() const
{
    static double maxAction = sin(g_eps / 2) * modelLength() * 2;
    return maxAction;
}
std::pair<double, double> ManipulatorPlanner::sitePosition(const JointState& state) const
{
    startProfiling();
    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = state.rad(i);
    }
    mj_forward(_model, _data);
    stopProfiling();
    return {_data->site_xpos[0], _data->site_xpos[1]};
}

void ManipulatorPlanner::initPrimitiveActions()
{
    _zeroAction = Action(_dof, 0);

    _primitiveActions.assign(2 * _dof, Action(_dof, 0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveActions[i][i] = 1;
        _primitiveActions[i + _dof][i] = -1;
    }
}

Solution ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    Solution solution(_primitiveActions, _zeroAction);

    JointState currentPos = startPos;
    for (size_t i = 0; i < _dof; ++i)
    {
        while (currentPos[i] != goalPos[i])
        {
            size_t t = -1;
            if (currentPos[i] < goalPos[i]) // + eps
            {
                t = i;
            }
            else if (currentPos[i] > goalPos[i]) // - eps
            {
                t = i + _dof;
            }

            if (checkCollisionAction(currentPos, _primitiveActions[t]))
            {
                solution.stats.pathVerdict = PATH_NOT_FOUND;
                return solution; // we temporary need to give up : TODO
            }
            currentPos.apply(_primitiveActions[t]);
            solution.addAction(t);
        }
    }

    solution.stats.pathVerdict = PATH_FOUND;
    return solution;
}

Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    float weight, double timeLimit
)
{
    AstarChecker checker(this, goalPos);
    Solution solution = astar::astar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}
Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, double goalX, double goalY,
    float weight, double timeLimit
)
{
    AstarCheckerSite checker(this, goalX, goalY);
    Solution solution = astar::astar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}

// Checkers

ManipulatorPlanner::AstarChecker::AstarChecker(ManipulatorPlanner* planner, const JointState& goal) : _goal(goal)
{
    _planner = planner;
}

bool ManipulatorPlanner::AstarChecker::isCorrect(const JointState& state, const Action& action)
{
    return state.applied(action).isCorrect() && (!_planner->checkCollisionAction(state, action));
}
bool ManipulatorPlanner::AstarChecker::isGoal(const JointState& state)
{
    return state == _goal;
}
CostType ManipulatorPlanner::AstarChecker::costAction(const Action& action)
{
    return action.abs();
}
const std::vector<Action>& ManipulatorPlanner::AstarChecker::getActions()
{
    return _planner->_primitiveActions;
}
const Action& ManipulatorPlanner::AstarChecker::getZeroAction()
{
    return _planner->_zeroAction;
}
CostType ManipulatorPlanner::AstarChecker::heuristic(const JointState& state)
{
    return manhattanHeuristic(state, _goal);
}

// checker for site goal


ManipulatorPlanner::AstarCheckerSite::AstarCheckerSite(ManipulatorPlanner* planner, double goalX, double goalY)
{
    _planner = planner;
    _goalX = goalX;
    _goalY = goalY;
}

bool ManipulatorPlanner::AstarCheckerSite::isCorrect(const JointState& state, const Action& action)
{
    return state.applied(action).isCorrect() && (!_planner->checkCollisionAction(state, action));
}
bool ManipulatorPlanner::AstarCheckerSite::isGoal(const JointState& state)
{
    const double r = 0.05; // const minimum dist from pos
    if (state.hasCacheXY())
    {
        double dx = state.cacheX() - _goalX;
        double dy = state.cacheY() - _goalY;
        return dx * dx + dy * dy <= r * r;
    }
    else
    {
        std::pair<double, double> xy = _planner->sitePosition(state);
        state.setCacheXY(xy.first, xy.second);
        double dx = xy.first - _goalX;
        double dy = xy.second - _goalY;
        return dx * dx + dy * dy <= r * r;
    }
}
CostType ManipulatorPlanner::AstarCheckerSite::costAction(const Action& action)
{
    return action.abs();
}
const std::vector<Action>& ManipulatorPlanner::AstarCheckerSite::getActions()
{
    return _planner->_primitiveActions;
}
const Action& ManipulatorPlanner::AstarCheckerSite::getZeroAction()
{
    return _planner->_zeroAction;
}
CostType ManipulatorPlanner::AstarCheckerSite::heuristic(const JointState& state)
{
    if (state.hasCacheXY())
    {
        double dx = state.cacheX() - _goalX;
        double dy = state.cacheY() - _goalY;
        return sqrt(dx * dx + dy * dy) / _planner->maxActionLength();
    }
    else
    {
        std::pair<double, double> xy = _planner->sitePosition(state);
        state.setCacheXY(xy.first, xy.second);
        double dx = xy.first - _goalX;
        double dy = xy.second - _goalY;
        return sqrt(dx * dx + dy * dy) / _planner->maxActionLength();
    }
}
