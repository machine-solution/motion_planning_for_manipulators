#pragma once

#include "joint_state.h"
#include "astar.h"
#include <mujoco/mujoco.h>

enum Algorithm
{
    ALG_LINEAR,
    ALG_ASTAR,
    ALG_MAX
};

struct Stats
{
    size_t expansions = 0;
    double runtime = 0.0;
    int pathCost = 0;
    size_t maxTreeSize = 0;
};

class ManipulatorPlanner
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model = nullptr, mjData* data = nullptr);

    JointState& nextStep();

    bool goalAchieved();

    bool checkCollision(const JointState& position);

    void planSteps(const JointState& startPos, const JointState& goalPos, int alg = ALG_MAX - 1);

    Stats stats() const;

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    void linearPlanning(const JointState& startPos, const JointState& goalPos);

    int costMove(const JointState& state1, const JointState& state2);
    // allocates on heap and returns successors
    vector<astar::SearchNode*> generateSuccessors(
        astar::SearchNode* node,
        const JointState& goal,
        int (*heuristicFunc)(const JointState& state1, const JointState& state2)
    );

    void astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        int (*heuristicFunc)(const JointState& state1, const JointState& state2)
    );

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    vector<size_t> _solveSteps; // vector id-s of primitiveSteps  
    size_t _nextStepId;
    size_t _dof;

    mjModel* _model; // model for collision checks
    mjData* _data; // data for collision checks

    Stats _stats;
};
