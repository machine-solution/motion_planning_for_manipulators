#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <stddef.h>
#include <initializer_list>
#include <cmath>

#include <mujoco/mujoco.h>

using std::vector;

class JointState
{
public:
    JointState(size_t dof = 2, double value = 0);
    JointState(std::initializer_list<double> list);
    
    double operator[](size_t i) const;
    double& operator[](size_t i);
    void operator+=(const JointState& other);

    double dist(const JointState& other);

private:
    vector<double> _joints;
    size_t _dof;
};

JointState randomState(size_t dof);

class ManipulatorPlanner
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model, mjData* data);
    ManipulatorPlanner(size_t dof, mjModel* model, mjData* data,
        const JointState& startPos, const JointState& endPos);

    JointState& nextStep();

    bool goalAchieved();

    bool checkCollision(const JointState& position);

    void planSteps(const JointState& startPos, const JointState& endPos);

private:
    void initPrimitiveSteps();

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    vector<size_t> _solveSteps; // vector id-s of primitiveSteps  
    size_t _nextStepId;
    size_t _dof;

    mjModel* _model; // model for collision checks
    mjData* _data; // data for collision checks

    const double _eps = 1.0 / 1024.0;
};

#endif
