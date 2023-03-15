#pragma once

#include "joint_state.h"
#include "astar.h"
#include "solution.h"
#include <mujoco/mujoco.h>

enum Algorithm
{
    ALG_LINEAR,
    ALG_ASTAR,
    ALG_MAX
};

class ManipulatorPlanner
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model = nullptr, mjData* data = nullptr);

    bool checkCollision(const JointState& position);

    Solution planSteps(const JointState& startPos, const JointState& goalPos, int alg = ALG_MAX - 1);

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    int costMove(const JointState& state1, const JointState& state2);
    // allocates on heap and returns successors
    vector<astar::SearchNode*> generateSuccessors(
        astar::SearchNode* node,
        const JointState& goal,
        int (*heuristicFunc)(const JointState& state1, const JointState& state2)
    );

    Solution astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        int (*heuristicFunc)(const JointState& state1, const JointState& state2)
    );

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    size_t _dof;

    mjModel* _model; // model for collision checks
    mjData* _data; // data for collision checks
};
