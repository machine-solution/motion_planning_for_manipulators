#include "planner.h"

#include <cmath>

#include <cstdio>

JointState::JointState(size_t dof, int value)
{
    _dof = dof;
    _joints.assign(_dof, value);
}
JointState::JointState(std::initializer_list<int> list)
{
    _joints.assign(list);
    _dof = _joints.size();
}

int JointState::operator[](size_t i) const
{
    // TODO 
    return _joints[i];
}
int& JointState::operator[](size_t i)
{
    // TODO
    return _joints[i];
}
JointState& JointState::operator=(const JointState& other)
{
    // TODO if (dof != other.dof)
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] = other._joints[i];
    }
    return *this;
}
JointState& JointState::operator+=(const JointState& other)
{
    // TODO if (dof != other.dof)
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] += other._joints[i];
    }
    return *this;
}

bool operator==(const JointState& state1, const JointState& state2)
{
    if (state1._dof != state2._dof)
    {
        return false;
    }
    for (size_t i = 0; i < state1._dof; ++i)
    {
        if (state1._joints[i] != state2._joints[i])
        {
            return false;
        }
    }

    return true;
}
bool operator!=(const JointState& state1, const JointState& state2)
{
    return !(state1 == state2);
}

double JointState::rad(size_t i)
{
    return eps * _joints[i];
}

JointState randomState(size_t dof, int units)
{
    JointState state(dof);
    for (size_t i = 0; i < dof; ++i)
    {
        state[i] = rand() % (units * 2) - M_PI - units + 1;
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
    if (_model == NULL || _data == NULL) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position[i] * eps;
    }
    mj_step1(_model, _data);
    return _data->ncon;
}

void ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& goalPos)
{
    linearPlanning(startPos, goalPos);
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

void ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    _nextStepId = 0;
    _solveSteps.clear();

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

            currentPos += _primitiveSteps[t];
            if (checkCollision(currentPos))
            {
                return; // we temporary need to give up : TODO
            }
            _solveSteps.push_back(t);
        }
    }
}
