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
    JointState(size_t dof = 2, int value = 0);
    JointState(std::initializer_list<int> list);
    
    int operator[](size_t i) const;
    int& operator[](size_t i);
    JointState& operator+=(const JointState& other);

    friend bool operator==(const JointState& state1, const JointState& state2);
    friend bool operator!=(const JointState& state1, const JointState& state2);

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

    const int units = 2048; // the number of units from [0, pi]
    const double eps = (M_PI / units); // length of 1 unit

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
