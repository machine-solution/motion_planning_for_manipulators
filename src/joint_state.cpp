#include "joint_state.h"

#include <algorithm>
#include <stdexcept>

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


Action::Action(size_t dof, int value)
{
    _dof = dof;
    _joints.assign(_dof, value);
}
Action::Action(std::initializer_list<int> list)
{
    _joints.assign(list);
    _dof = _joints.size();
}

size_t Action::dof() const
{
    return _dof;
}
int Action::operator[](size_t i) const
{
    // TODO
    return _joints[i];
}
int& Action::operator[](size_t i)
{
    // TODO
    return _joints[i];
}

int Action::abs() const
{
    int len = 0;
    for (size_t i = 0; i < dof(); ++i)
    {
        len += std::abs(_joints[i]); // It is integer abs
    }
    return len;
}

size_t Action::byteSize() const
{
    return sizeof(_dof) + sizeof(int) * _joints.size();
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
    _hasCacheXY = false;
    // TODO
    return _joints[i];
}

JointState& JointState::apply(const Action& action)
{
    _hasCacheXY = false;
    if (_dof != action.dof())
    {
        throw std::runtime_error("JointState::apply: dofs of operands are not equal");
    }
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] += action[i];
    }
    _lastAction = &action;
    normalize();
    return *this;
}
JointState JointState::applied(const Action& action) const
{
    JointState result = *this;
    return result.apply(action);
}

JointState& JointState::operator=(const JointState& other)
{
    _hasCacheXY = false;
    _dof = other._dof;
    _joints.resize(_dof);
    for (size_t i = 0; i < _dof; ++i)
    {
        _joints[i] = other._joints[i];
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
    return state2 < state1;
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

const Action* JointState::lastAction() const
{
    return _lastAction;
}

int JointState::maxJoint() const
{
    return *std::max_element(_joints.begin(), _joints.end());
}
int JointState::minJoint() const
{
    return *std::min_element(_joints.begin(), _joints.end());
}

int JointState::abs() const
{
    int len = 0;
    for (size_t i = 0; i < dof(); ++i)
    {
        len += std::abs(_joints[i]); // It is integer abs
    }
    return len;
}

bool JointState::isCorrect() const
{
    return minJoint() >= -g_units && maxJoint() < g_units;
}

size_t JointState::byteSize() const
{
    return _joints.size() * sizeof(int) + sizeof(_dof) + sizeof(Action*) +
           sizeof(_cacheX) + sizeof(_cacheY) + sizeof(_hasCacheXY);
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
int manhattanDistance(const Action& action1, const Action& action2)
{
    int dist = 0;
    for (size_t i = 0; i < action1.dof(); ++i)
    {
        dist += abs(action1[i] - action2[i]);
    }
    return dist;
}

CostType manhattanHeuristic(const JointState& state1, const JointState& state2)
{
    return manhattanDistance(state1, state2);
}

bool JointState::hasCacheXY() const
{
    return _hasCacheXY;
}
double JointState::cacheX() const
{
    return _cacheX;
}
double JointState::cacheY() const
{
    return _cacheY;
}
void JointState::setCacheXY(double x, double y) const
{
    _hasCacheXY = true;
    _cacheX = x;
    _cacheY = y;
}

void JointState::normalize()
{
    if (_dof > 0)
    {
        _joints[0] = trueMod(_joints[0], g_units);
    }
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

MultiAction::MultiAction(size_t dof, size_t arms, int value)
{
    _dof = dof;
    _arms = arms;
    _actions = std::vector(_arms, Action(_dof, value));
}

MultiAction::MultiAction(std::vector<Action> actions)
{
    _arms = actions.size();
    _dof = 2;
    if (_arms > 0)
    {
        _dof = actions[0].dof();
    }
    _actions = actions;
}

size_t MultiAction::dof() const
{
    return _dof;
}

size_t MultiAction::arms() const
{
    return _arms;
}

Action MultiAction::operator[](size_t i) const
{
    return _actions.at(i);
}

Action &MultiAction::operator[](size_t i)
{
    return _actions.at(i);
}

MultiState::MultiState(size_t dof, size_t arms, int value)
{
    _dof = dof;
    _arms = arms;
    _states = std::vector<JointState>(arms, JointState(dof, value));
}

MultiState::MultiState(std::vector<JointState> states)
{
    _arms = states.size();
    _dof = 2;
    if (_arms > 0)
    {
        _dof = states[0].dof();
    }
    _states = states;
}

size_t MultiState::dof() const
{
    return _dof;
}

size_t MultiState::arms() const
{
    return _dof;
}

JointState MultiState::operator[](size_t i) const
{
    return _states.at(i);
}

JointState& MultiState::operator[](size_t i)
{
    return _states.at(i);
}

MultiState& MultiState::apply(const MultiAction &action)
{
    if (_dof != action.dof())
    {
        throw std::runtime_error("JointState::apply: dofs of operands are not equal");
    }
    if (_arms != action.arms())
    {
        throw std::runtime_error("JointState::apply: arms of operands are not equal");
    }
    for (size_t a = 0; a < _arms; ++a)
    {
        _states[a].apply(action[a]);
    }
    return *this;
}

MultiState MultiState::applied(const MultiAction &action) const
{
    MultiState result = *this;
    return result.apply(action);
}

MultiState &MultiState::operator=(const MultiState &other)
{
    _dof = other._dof;
    _arms = other._arms;

    _states.resize(_arms);

    for (size_t a = 0; a < _arms; ++a)
    {
        _states[a] = other[a];
    }
    return *this;
}
