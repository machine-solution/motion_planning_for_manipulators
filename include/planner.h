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
    ALG_MAX,
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

    // timeLimit - is a maximum time in *seconds*, after that planner will give up
    Solution planSteps(const JointState& startPos, const JointState& goalPos, double timeLimit = 1.0, int alg = ALG_MAX - 1);

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    Solution astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
        float weight, double timeLimit
    );

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    size_t _dof;

    mutable mjModel* _model; // model for collision checks
    mutable mjData* _data; // data for collision checks

    class AstarChecker : public astar::IAstarChecker
    {
    public:
        AstarChecker(ManipulatorPlanner* planner);

        bool isCorrect(const JointState& state, const JointState& action) override;
        CostType costAction(const JointState& action) override;
        const std::vector<JointState>& getActions() override;
        const JointState& getZeroAction() override;
    private:
        ManipulatorPlanner* _planner;
    };
};
