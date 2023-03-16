#include "joint_state.h"

#include <algorithm>

// return n % (2 * mod) in [-mod, mod)
int trueMod(int n, int mod)
{
    n = n % (2 * mod);
    if (n < -mod)
    {
        return n + 2 * mod;
    }
    else if (n >= mod)
    {
        return n - 2 * mod;
    }
    else
    {
        return n;
    }
}

JointState::JointState(size_t dof, int value)
{
    _dof = dof;
    _joints.assign(_dof, value);
    normalize();
}
JointState::JointState(std::initializer_list<int> list)
{
    _joints.assign(list);
    _dof = _joints.size();
    normalize();
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
    normalize();
    return *this;
}
JointState& JointState::operator+=(const JointState& other)
{
    // TODO if (dof != other.dof)
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] += other._joints[i];
    }
    normalize();
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
    return g_eps * _joints[i];
}

size_t JointState::dof() const
{
    return _dof;
}

int JointState::maxJoint() const
{
    return *std::max_element(_joints.begin(), _joints.end());
}
int JointState::minJoint() const
{
    return *std::min_element(_joints.begin(), _joints.end());
}

bool JointState::isCorrect() const
{
    return minJoint() >= -g_units && maxJoint() < g_units;
}

int manhattanDistance(const JointState& state1, const JointState& state2)
{
    int dist = 0;
    for (size_t i = 0; i < state1.dof(); ++i)
    {
        if (i == 0)
        {
            dist += std::min(abs(state1[i] - state2[i]), 2 * g_units - abs(state1[i] - state2[i]));
        }
        else
        {
            dist += abs(state1[i] - state2[i]);
        }
    }
    return dist;
}

void JointState::normalize()
{
    _joints[0] = trueMod(_joints[0], g_units);
}

JointState randomState(size_t dof, int units)
{
    JointState state(dof);
    for (size_t i = 0; i < dof; ++i)
    {
        state[i] = rand() % (units * 2) - units;
    }
    return state;
}
