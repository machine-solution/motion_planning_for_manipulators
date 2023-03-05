#include "planner.h"

#include <cmath>

#include <cstdio>

JointState::JointState(size_t dof, double value)
{
    _dof = dof;
    _joints.assign(_dof, value);
}
JointState::JointState(std::initializer_list<double> list)
{
    _joints.assign(list);
    _dof = _joints.size();
}

double JointState::operator[](size_t i) const
{
    // TODO 
    return _joints[i];
}
double& JointState::operator[](size_t i)
{
    // TODO
    return _joints[i];
}
void JointState::operator+=(const JointState& other)
{
    // TODO if (dof != other.dof)
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] += other._joints[i];
    }
}

double JointState::dist(const JointState& other)
{
    double dist = 0;
    for (size_t i = 0; i < _dof; ++i)
    {
        dist = std::max(dist, fabs(_joints[i] - other._joints[i]));
    }
    return dist;
}

JointState randomState(size_t dof)
{
    JointState state(dof);
    for (size_t i = 0; i < dof; ++i)
    {
        state[i] = 2 * M_PI * rand() / RAND_MAX - M_PI;
    }
    return state;
}


ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveSteps();
}
ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data,
    const JointState& startPos, const JointState& endPos) : ManipulatorPlanner(dof, model, data)
{
    planSteps(startPos, endPos);
}

JointState& ManipulatorPlanner::nextStep()
{
    if (_nextStepId >= _solveSteps.size())
    {
        return _zeroStep;
    }
    return _primitiveSteps[_solveSteps[_nextStepId++]];
}

bool ManipulatorPlanner::goalAchieved()
{
    return _nextStepId >= _solveSteps.size();
}

bool ManipulatorPlanner::checkCollision(const JointState& position)
{
    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position[i];
    }
    mj_step1(_model, _data);
    return _data->ncon;
}

void ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& endPos)
{
    _nextStepId = 0;
    _solveSteps.clear();

    JointState currentPos = startPos;
    for (size_t i = 0; i < _dof; ++i)
    {
        while (fabs(currentPos[i] - endPos[i]) > _eps / 2)
        {
            size_t t = -1;
            if (currentPos[i] < endPos[i]) // + eps
            {
                t = i;
            }
            else if (currentPos[i] > endPos[i]) // - eps
            {
                t = i + _dof;
            }

            currentPos += _primitiveSteps[t];
            if (checkCollision(currentPos))
            {
                return; // we temporary need to give up : TODO
            }
            _solveSteps.push_back(t);
        }
    }
}

void ManipulatorPlanner::initPrimitiveSteps()
{
    _zeroStep = JointState(_dof, 0.0);

    _primitiveSteps.assign(2 * _dof, JointState(_dof, 0.0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveSteps[i][i] = _eps;
        _primitiveSteps[i + _dof][i] = -_eps;
    }
}
