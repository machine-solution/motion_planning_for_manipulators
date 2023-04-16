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
    initPrimitiveSteps();
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

bool ManipulatorPlanner::checkCollisionAction(const JointState& start, const JointState& delta) const
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
            _data->qpos[i] = start.rad(i) + g_worldEps * delta[i] * t; // temporary we use global constant here for speed
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

Solution ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& goalPos, int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(goalPos))
    {
        Solution solution(_primitiveSteps, _zeroStep);
        solution.stats.pathVerdict = PATH_NOT_EXISTS; // incorrect aim
        return  solution;
    }
    switch (alg)
    {
    case ALG_LINEAR:
        return linearPlanning(startPos, goalPos);
    case ALG_ASTAR:
        return astarPlanning(startPos, goalPos, manhattanHeuristic, w, timeLimit);
    default:
        return Solution(_primitiveSteps, _zeroStep);
    }
}

void ManipulatorPlanner::initPrimitiveSteps()
{
    _zeroStep = JointState(_dof, 0);

    _primitiveSteps.assign(2 * _dof, JointState(_dof, 0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveSteps[i][i] = 1;
        _primitiveSteps[i + _dof][i] = -1;
    }
}

Solution ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    Solution solution(_primitiveSteps, _zeroStep);

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

            if (checkCollisionAction(currentPos, _primitiveSteps[t]))
            {
                solution.stats.pathVerdict = PATH_NOT_FOUND;
                return solution; // we temporary need to give up : TODO
            }
            currentPos += _primitiveSteps[t];
            solution.addStep(t);
        }
    }

    solution.stats.pathVerdict = PATH_FOUND;
    return solution;
}

Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
    float weight, double timeLimit
)
{
    AstarChecker checker(this);
    Solution solution = astar::astar(startPos, goalPos, checker, heuristicFunc, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}


ManipulatorPlanner::AstarChecker::AstarChecker(ManipulatorPlanner* planner)
{
    _planner = planner;
}

bool ManipulatorPlanner::AstarChecker::isCorrect(const JointState& state, const JointState& action)
{
    return (state + action).isCorrect() && (!_planner->checkCollisionAction(state, action));
}
CostType ManipulatorPlanner::AstarChecker::costAction(const JointState& action)
{
    return action.abs();
}
const std::vector<JointState>& ManipulatorPlanner::AstarChecker::getActions()
{
    return _planner->_primitiveSteps;
}
const JointState& ManipulatorPlanner::AstarChecker::getZeroAction()
{
    return _planner->_zeroStep;
}

