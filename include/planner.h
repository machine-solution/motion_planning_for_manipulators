#pragma once

#include "astar.h"
#include "joint_state.h"
#include "lazy_astar.h"
#include "solution.h"
#include "utils.h"
#include <mujoco/mujoco.h>

#include <map>

enum Algorithm
{
    ALG_LINEAR,
    ALG_ASTAR,
    ALG_LAZY_ASTAR,
    ALG_MAX,
};

class PreprocData
{
public:
    PreprocData();

    size_t byteSize() const;
    size_t kbyteSize() const;
    size_t mbyteSize() const;

    std::map<JointState, size_t> actionsMap; // state -> action's index
    JointState homeState;
    bool isPreprocessed = false;
};

class ManipulatorPlanner : public Profiler
{
public:
    ManipulatorPlanner(size_t dof, mjModel* model = nullptr, mjData* data = nullptr);

    size_t dof() const;

    bool checkCollision(const JointState& position) const;
    bool checkCollisionAction(const JointState& start, const Action& action) const;

    // return C-Space as strings where @ an obstacle, . - is not
    // only for _dof = 2 now
    vector<string> configurationSpace() const;

    // On C-Space print start as 'A', end as 'B', path as '+'
    // Make copy of solution to call nextStep()
    vector<string> pathInConfigurationSpace(const JointState& start, Solution solution) const;

    // timeLimit - is a maximum time in *seconds*, after that planner will give up
    Solution planActions(const JointState& startPos, const JointState& goalPos, int alg = ALG_MAX - 1,
        double timeLimit = 1.0, double w = 1.0);
    // plan path to move end-effector to (doubleX, doubleY) point
    // timeLimit - is a maximum time in *seconds*, after that planner will give up
    Solution planActions(const JointState& startPos, double goalX, double goalY, int alg = ALG_ASTAR,
        double timeLimit = 1.0, double w = 1.0);
    
    void preprocess();
    bool isPreprocessed() const;

    // this method used that edges of model are cylinders
    // and that manipulator has geom numbers 1 .. _dof inclusively
    // calculate length only 1 time
    double modelLength() const;
    // calculate answer only 1 time
    double maxActionLength() const;
    // return coords of site by state of joints
    std::pair<double, double> sitePosition(const JointState& state) const;

    const int units = g_units;
    const double eps = g_eps;

    const vector<Action>& getPrimitiveActions() const;

private:
    void initPrimitiveActions();

    JointState sampleFreeState(int attempts = 0);

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    Solution astarPlanning(
        const JointState& startPos, const JointState& goalPos,
        float weight, double timeLimit
    );
    Solution astarPlanning(
        const JointState& startPos, double goalX, double goalY,
        float weight, double timeLimit
    );

    Solution lazyAstarPlanning(
        const JointState& startPos, const JointState& goalPos,
        float weight, double timeLimit
    );
    Solution lazyAstarPlanning(
        const JointState& startPos, double goalX, double goalY,
        float weight, double timeLimit
    );

    Solution preprocPlanning(
        const JointState& startPos, const JointState& goalPos
    );

    vector<Action> _primitiveActions;
    Action _zeroAction;
    size_t _dof;

    PreprocData _preprocData;

    mutable mjModel* _model; // model for collision checks
    mutable mjData* _data; // data for collision checks and calculations

    class AstarChecker : public astar::IAstarChecker
    {
    public:
        AstarChecker(ManipulatorPlanner* planner, const JointState& goal);

        bool isCorrect(const JointState& state, const Action& action) override;
        bool isGoal(const JointState& state) override;
        CostType costAction(const JointState& state, const Action& action) override;
        const std::vector<Action>& getActions() override;
        const Action& getZeroAction() override;
        CostType heuristic(const JointState& state) override;
    protected:
        ManipulatorPlanner* _planner;
        const JointState& _goal;
    };

    class AstarCheckerSite : public astar::IAstarChecker
    {
    public:
        AstarCheckerSite(ManipulatorPlanner* planner, double goalX, double goalY);

        bool isCorrect(const JointState& state, const Action& action) override;
        bool isGoal(const JointState& state) override;
        CostType costAction(const JointState& state, const Action& action) override;
        const std::vector<Action>& getActions() override;
        const Action& getZeroAction() override;
        CostType heuristic(const JointState& state) override;
    protected:
        ManipulatorPlanner* _planner;
        double _goalX;
        double _goalY;
    };
};
