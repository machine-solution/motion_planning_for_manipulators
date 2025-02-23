#pragma once

#include "joint_state.h"
#include "solution.h"

#include <string>
#include <memory>

enum ConstraintType
{
    CONSTRAINT_VERTEX,
    CONSTRAINT_SPHERE,
    CONSTRAINT_AVOIDANCE,
    CONSTRAINT_PRIORITY,

    CONSTRAINT_MAX // the number of Constraint types
};

class Conflict {
public:
    Conflict(
        size_t firstArmNum, JointState firstPreviousState, Action firstAction,
        size_t secondArmNum, JointState secondPreviousState, Action secondAction,
        int stepNum, std::vector<double> point
    );
    Conflict();

    bool has() const;
    int stepNum() const;
    size_t firstArm() const;
    size_t secondArm() const;
    JointState firstPreviousState() const;
    JointState secondPreviousState() const;
    JointState firstState() const;
    JointState secondState() const;
    Action firstAction() const;
    Action secondAction() const;
    std::vector<double> point() const;
private:

    size_t _firstArmNum;
    JointState _firstPreviousState;
    Action _firstAction;
    size_t _secondArmNum;
    JointState _secondPreviousState;
    Action _secondAction;
    int _stepNum;
    bool _has;
    std::vector<double> _point;
};

class IConstraint
{
public:
    virtual ConstraintType type() const = 0;
};

using ConstraintList = std::vector<std::shared_ptr<IConstraint>>;

class EmptyConstraint : public IConstraint
{
public:
    EmptyConstraint();
    
    ConstraintType type() const override;
};

class VertexConstraint : public IConstraint
{
public:
    VertexConstraint(int stepFrom, int stepTo, const JointState& state);

    int stepFrom() const;
    int stepTo() const;
    JointState state() const;
    
    ConstraintType type() const override;
private:
    int _stepFrom;
    int _stepTo;
    JointState _state;
};

class SphereConstraint : public IConstraint
{
public:
    SphereConstraint(int stepFrom, int stepTo, double centerX, double centerY, double centerZ, double radius = 0.1);

    int stepFrom() const;
    int stepTo() const;
    const double centerX() const;
    const double centerY() const;
    const double centerZ() const;
    const double radius() const;
    
    ConstraintType type() const override ;
private:
    int _stepFrom;
    int _stepTo;
    double _centerX;
    double _centerY;
    double _centerZ;
    double _radius;
};

class AvoidanceConstraint : public IConstraint
{
public:
    AvoidanceConstraint(int stepFrom, int stepTo, size_t armNum, const JointState& state);

    int stepFrom() const;
    int stepTo() const;
    const size_t armNum() const;
    const JointState& state() const;
    
    ConstraintType type() const override ;
private:
    int _stepFrom;
    int _stepTo;
    size_t _armNum;
    JointState _state;
};

class PriorityConstraint : public IConstraint
{
public:
    PriorityConstraint(size_t armNum);

    size_t armNum() const;
    
    ConstraintType type() const override ;
private:
    size_t _armNum;
};


struct ConstraintSet
{
public:
    ConstraintSet(const MultiState& startState, MultiSolution solution, ConstraintList allConstraints);

    ConstraintList constraints;
    std::vector<StateChain> stateChains;

    size_t arms;
};
