#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <stddef.h>
#include <initializer_list>
#include <cmath>

#include <mujoco/mujoco.h>

using std::vector;

namespace {
const int g_units = 2048; // the number of units from [0, pi]
const double g_eps = (M_PI / g_units); // length of 1 unit
}

class JointState
{
public:
    JointState(size_t dof = 2, int value = 0);
    JointState(std::initializer_list<int> list);
    
    int operator[](size_t i) const;
    int& operator[](size_t i);
    JointState& operator=(const JointState& other);
    JointState& operator+=(const JointState& other);

    friend bool operator==(const JointState& state1, const JointState& state2);
    friend bool operator!=(const JointState& state1, const JointState& state2);

    // returns angle of i-th joint in radians (-pi, pi]
    double rad(size_t i);

    const int units = g_units;
    const double eps = g_eps;

private:
    vector<int> _joints;
    size_t _dof;
};

JointState randomState(size_t dof, int units);

class ManipulatorPlanner
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model = NULL, mjData* data = NULL);

    JointState& nextStep();

    bool goalAchieved();

    bool checkCollision(const JointState& position);

    void planSteps(const JointState& startPos, const JointState& endPos);

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    vector<size_t> _solveSteps; // vector id-s of primitiveSteps  
    size_t _nextStepId;
    size_t _dof;

    mjModel* _model; // model for collision checks
    mjData* _data; // data for collision checks
};

#endif
