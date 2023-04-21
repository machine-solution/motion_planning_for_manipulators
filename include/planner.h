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
    Solution planSteps(const JointState& startPos, const JointState& goalPos, int alg = ALG_MAX - 1,
        double timeLimit = 1.0, double w = 1.0);

    // this method used that edges of model are cylinders
    // and that manipulator has geom numbers 1 .. _dof inclusively
    double modelLength() const;
    double maxStepLen() const;

    const int units = g_units;
    const double eps = g_eps;

private:
    void initPrimitiveSteps();

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    Solution astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        float weight, double timeLimit
    );

    vector<JointState> _primitiveSteps;
    JointState _zeroStep;
    size_t _dof;

    mutable mjModel* _model; // model for collision checks
    mutable mjData* _data; // data for collision checks and calculations

    class AstarChecker : public astar::IAstarChecker
    {
    public:
        AstarChecker(ManipulatorPlanner* planner, const JointState& goal);

        bool isCorrect(const JointState& state, const JointState& action) override;
        bool isGoal(const JointState& state) override;
        CostType costAction(const JointState& action) override;
        const std::vector<JointState>& getActions() override;
        const JointState& getZeroAction() override;
        CostType heuristic(const JointState& state) override;
    protected:
        ManipulatorPlanner* _planner;
        const JointState& _goal;
    };

    class AstarCheckerSite : public AstarChecker
    {
    public:
        AstarCheckerSite(ManipulatorPlanner* planner, const JointState& goal);
        bool isGoal(const JointState& state) override;
        CostType heuristic(const JointState& state) override;
    };
};
