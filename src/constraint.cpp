#include "constraint.h"

#include <iostream>

Conflict::Conflict(size_t firstArmNum, JointState firstPreviousState, Action firstAction, size_t secondArmNum, JointState secondPreviousState, Action secondAction, int stepNum, std::vector<double> point)
{
    _firstArmNum = firstArmNum;
    _secondArmNum = secondArmNum;
    _firstPreviousState = firstPreviousState;
    _secondPreviousState = secondPreviousState;
    _firstAction = firstAction;
    _secondAction = secondAction;
    _stepNum = stepNum;
    _point = point;

    _has = true;
}

Conflict::Conflict()
{
    _has = false;
}

int Conflict::stepNum() const
{
    return _stepNum;;
}

bool Conflict::has() const
{
    return _has;
}

size_t Conflict::firstArm() const
{
    return _firstArmNum;
}

size_t Conflict::secondArm() const
{
    return _secondArmNum;
}

JointState Conflict::firstPreviousState() const
{
    return _firstPreviousState;
}

JointState Conflict::secondPreviousState() const
{
    return _secondPreviousState;
}

JointState Conflict::firstState() const
{
    return _firstPreviousState.applied(_firstAction);
}

JointState Conflict::secondState() const
{
    return _secondPreviousState.applied(_secondAction);
}

Action Conflict::firstAction() const
{
    return _firstAction;
}

Action Conflict::secondAction() const
{
    return _secondAction;
}

std::vector<double> Conflict::point() const
{
    return _point;
}

EmptyConstraint::EmptyConstraint()
{
}

ConstraintType EmptyConstraint::type() const
{
    return CONSTRAINT_MAX;
}

VertexConstraint::VertexConstraint(int stepFrom, int stepTo, const JointState &state)
{
    _stepFrom = stepFrom;
    _stepTo = stepTo;
    _state = state;
}

int VertexConstraint::stepFrom() const
{
    return _stepFrom;
}

int VertexConstraint::stepTo() const
{
    return _stepTo;
}

JointState VertexConstraint::state() const
{
    return _state;
}

ConstraintType VertexConstraint::type() const
{
    return CONSTRAINT_VERTEX;
}

AvoidanceConstraint::AvoidanceConstraint(int stepFrom, int stepTo, size_t armNum, const JointState &state)
{
    _stepFrom = stepFrom;
    _stepTo = stepTo;
    _armNum = armNum;
    _state = state;
}

int AvoidanceConstraint::stepFrom() const
{
    return _stepFrom;
}

int AvoidanceConstraint::stepTo() const
{
    return _stepTo;
}

const size_t AvoidanceConstraint::armNum() const
{
    return _armNum;
}

const JointState &AvoidanceConstraint::state() const
{
    return _state;
}

ConstraintType AvoidanceConstraint::type() const
{
    return CONSTRAINT_AVOIDANCE;
}

SphereConstraint::SphereConstraint(int stepFrom, int stepTo, double centerX, double centerY, double centerZ, double radius)
{
    _stepFrom = stepFrom;
    _stepTo = stepTo;
    _centerX = centerX;
    _centerY = centerY;
    _centerZ = centerZ;
    _radius = radius;
}

int SphereConstraint::stepFrom() const
{
    return _stepFrom;
}

int SphereConstraint::stepTo() const
{
    return _stepTo;
}

const double SphereConstraint::centerX() const
{
    return _centerX;
}

const double SphereConstraint::centerY() const
{
    return _centerY;
}

const double SphereConstraint::centerZ() const
{
    return _centerZ;
}

const double SphereConstraint::radius() const
{
    return _radius;
}

ConstraintType SphereConstraint::type() const
{
    return CONSTRAINT_SPHERE;
}

PriorityConstraint::PriorityConstraint(size_t armNum)
{
}

size_t PriorityConstraint::armNum() const
{
    return size_t();
}

ConstraintType PriorityConstraint::type() const
{
    return CONSTRAINT_PRIORITY;
}

ConstraintSet::ConstraintSet(const MultiState &startState, MultiSolution solution, ConstraintList allConstraints)
{
    arms = startState.arms();
    constraints = allConstraints;
    for (size_t a = 0; a < arms; ++a)
    {
        stateChains.push_back(StateChain(startState[a], solution[a]));
    }
}
