#include "joint_state.h"

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

JointState JointState::operator+(const JointState& other) const
{
    JointState result = *this;
    return (result += other);
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

bool operator<(const JointState& state1, const JointState& state2)
{
    if (state1._dof != state2._dof)
    {
        return state1._dof < state2._dof;
    }
    return state1._joints < state2._joints;
}
bool operator>(const JointState& state1, const JointState& state2)
{
    return !((state1 < state2) || (state1 == state2));
}
bool operator<=(const JointState& state1, const JointState& state2)
{
    return !(state1 > state2);
}
bool operator>=(const JointState& state1, const JointState& state2)
{
    return !(state1 < state2);
}
bool operator==(const JointState& state1, const JointState& state2)
{
    if (state1._dof != state2._dof)
    {
        return false;
    }
    return state1._joints == state2._joints;
}
bool operator!=(const JointState& state1, const JointState& state2)
{
    return !(state1 == state2);
}

double JointState::rad(size_t i) const
{
    return eps * _joints[i];
}

size_t JointState::dof() const
{
    return _dof;
}

int manhattanDistance(const JointState& state1, const JointState& state2)
{
    int dist = 0;
    for (size_t i = 0; i < state1.dof(); ++i)
    {
        dist += abs(state1[i] - state2[i]);
    }
    return dist;
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
