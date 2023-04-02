#pragma once

#include "joint_state.h"
#include "astar.h"
#include "solution.h"
#include "utils.h"
#include <mujoco/mujoco.h>

enum Algorithm
{
    ALG_LINEAR,
    ALG_ASTAR,
    ALG_MAX
};

class ManipulatorPlanner : public Profiler
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model = nullptr, mjData* data = nullptr);

    size_t dof() const;

    bool checkCollision(const JointState& position) const;
    bool checkCollisionAction(const JointState& start, const JointState& delta) const;

    // return C-Space as strings where @ an obstacle, . - is not
    // only for _dof = 2 now
    vector<string> configurationSpace() const;

    Solution planSteps(const JointState& startPos, const JointState& goalPos, int alg = ALG_MAX - 1);

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    CostType costMove(const JointState& state1, const JointState& state2) const;
    // allocates on heap and returns successors
    vector<astar::SearchNode*> generateSuccessors(
        astar::SearchNode* node,
        const JointState& goal,
        CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
        float weight
    ) const;

    Solution astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
        float weight = 1
    );

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    size_t _dof;

    mutable mjModel* _model; // model for collision checks
    mutable mjData* _data; // data for collision checks
};
