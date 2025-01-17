#pragma once

#include "joint_state.h"
#include "planner.h"

#include <string>
#include <memory>

enum ConstraintType
{
    CONSTRAINT_VERTEX,
    CONSTRAINT_EDGE,
    CONSTRAINT_SPHERE,
    CONSTRAINT_AVOIDANCE,
    CONSTRAINT_PRIORITY,
    CONSTRAINT_MAX // the number of Constraint types
};

class IConstraint
{
public:
    virtual ConstraintType type() const = 0;
};

class VertexConstraint : public IConstraint
{
public:
    VertexConstraint(size_t stepNum, const JointState& position);

    const size_t stepNum() const;
    const JointState& position() const;
    
    ConstraintType type() const override;
private:
    size_t _stepNum;
    JointState _position;
};

class EdgeConstraint : public IConstraint
{
public:
    EdgeConstraint(size_t stepNum, const JointState& position, const Action& action);

    const size_t stepNum() const;
    const JointState& position() const;
    const Action& action() const;
    
    ConstraintType type() const override;
private:
    size_t _stepNum;
    JointState _position;
    Action _action;
};

class SphereConstraint : public IConstraint
{
public:
    SphereConstraint(size_t stepNum, double centerX, double centerY, double centerZ, double radius);

    const size_t stepNum() const;
    const double centerX() const;
    const double centerY() const;
    const double centerZ() const;
    const double radius() const;
    
    ConstraintType type() const override;
private:
    size_t _stepNum;
    double _centerX;
    double _centerY;
    double _centerZ;
    double _radius;
};

class AvoidanceConstraint : public IConstraint
{
public:
    AvoidanceConstraint(size_t stepNum, size_t armNum, const JointState& position);

    const size_t stepNum() const;
    const size_t armNum() const;
    const JointState& position() const;
    
    ConstraintType type() const override;
private:
    size_t _stepNum;
    size_t _armNum;
    JointState _position;
};

