#pragma once

#include "astar.h"
#include "constraint.h"
#include "joint_state.h"
#include "lazy_astar.h"
#include "lazy_arastar.h"
#include "solution.h"
#include "utils.h"
#include <mujoco/mujoco.h>

#include <map>

enum Algorithm
{
    ALG_LINEAR,
    ALG_ASTAR,
    ALG_LAZY_ASTAR,
    ALG_PREPROC_CLUSTERS,
    ALG_ARASTAR,
    ALG_PREPROC_ARASTAR,
    ALG_MAX,
};

enum Preprocess
{
    PRE_NONE,
    PRE_CLUSTERS,
    PRE_MAX,
};

class Cluster
{
public:
    Cluster(const JointState& center);

    int dist(const JointState& state) const;

    void setCenter(const JointState& center);
    JointState getCenter() const;

    void setSolution(const Solution& solution);
    Solution getSolution() const;

    size_t byteSize() const;
private:
    JointState _center;
    Solution _solution;
};

class PreprocData
{
public:
    PreprocData();

    size_t byteSize() const;
    size_t kbyteSize() const;
    size_t mbyteSize() const;

    std::vector<Cluster> clusters;
    JointState homeState;
    bool isPreprocessed = false;
    double preprocRuntime = 0.0;
};

class ManipulatorPlanner : public Profiler
{
public:
    ManipulatorPlanner(size_t dof, size_t arms, mjModel* model = nullptr, mjData* data = nullptr);

    size_t dof() const;
    size_t arms() const;

    void switchArm(size_t armNum, int mode) const;
    void switchSphere(int mode) const;
    void onArmsOnly(std::set<size_t> onArms) const;
    void offArmsOnly(std::set<size_t> offArms) const;

    void setArmState(size_t armNum, const JointState& state) const;
    void setSphereState(double centerX, double centerY, double centerZ, double radius) const;

    bool checkCollisionActionObstacles(
        size_t armNum, const JointState& start, const Action& action) const;
    bool checkCollisionActionConstraintVertex(
        size_t armNum, const JointState& start, int stepNum, const Action& action,
        std::shared_ptr<VertexConstraint> constraint) const;
    bool checkCollisionActionConstraintAvoidance(
        size_t armNum, const JointState& start, int stepNum, const Action& action,
        std::shared_ptr<AvoidanceConstraint> constraint) const;
    bool checkCollisionActionConstraintSphere(
        size_t armNum, const JointState& start, int stepNum, const Action& action,
        std::shared_ptr<SphereConstraint> constraint) const;
    bool checkCollisionActionConstraintPriority(
        size_t armNum, const JointState& start, int stepNum, const Action& action,
        std::shared_ptr<PriorityConstraint> constraint, const StateChain& states) const;

    bool checkCollisionStayForeverConstraintVertex(
        size_t armNum, const JointState& start, int stepNum,
        std::shared_ptr<VertexConstraint> constraint) const;
    bool checkCollisionStayForeverConstraintAvoidance(
        size_t armNum, const JointState& start, int stepNum,
        std::shared_ptr<AvoidanceConstraint> constraint) const;
    bool checkCollisionStayForeverConstraintSphere(
        size_t armNum, const JointState& start, int stepNum,
        std::shared_ptr<SphereConstraint> constraint) const;
    bool checkCollisionStayForeverConstraintPriority(
        size_t armNum, const JointState& start, int stepNum,
        std::shared_ptr<PriorityConstraint> constraint, const StateChain& states) const;

    bool checkCollision(size_t armNum, const JointState& position) const;
    bool checkMultiCollision(const MultiState& positions) const;
    bool checkCollisionAction(
        size_t armNum, const JointState& start, int stepNum, const Action& action,
        const ConstraintSet& constraints) const;
    // stepNum is a step directly after last step in solution
    // function checks that staying all steps from [stepNum, inf) with zero action are correct
    bool checkCollisionStayForever(
        size_t armNum, const JointState& start, int stepNum,
        const ConstraintSet& constraints) const;
    bool checkMultiCollisionAction(
        const MultiState& start, int stepNum, const MultiAction& action) const;
    
    std::vector<double> findIntersectionPoint(size_t armNum1, size_t armNum2, const JointState& state1, const JointState& state2) const;
    Conflict findFirstConflict(MultiState startPos, MultiSolution solution) const;
    size_t calculateConflictsCount(MultiState startPos, MultiSolution solution) const;

    // return C-Space as strings where @ an obstacle, . - is not
    // only for _dof * _arms = 2 now
    vector<string> configurationSpace() const;

    // On C-Space print start as 'A', end as 'B', path as '+'
    // Make copy of solution to call nextStep()
    vector<string> pathInConfigurationSpace(const JointState& start, Solution solution) const;

    MultiSolution planMultiActions(
        const MultiState& startPos, const MultiState& goalPos, int alg = ALG_ASTAR,
        double timeLimit = 1.0, double w = 1.0);

    // timeLimit - is a maximum time in *seconds*, after that planner will give up
    Solution planActions(size_t armNum, const JointState& startPos, const JointState& goalPos,
        const ConstraintSet& constraints, int alg = ALG_ASTAR,
        double timeLimit = 1.0, double w = 1.0);
    // plan path to move end-effector to (doubleX, doubleY) point
    // timeLimit - is a maximum time in *seconds*, after that planner will give up
    Solution planActions(size_t armNum, const JointState& startPos, double goalX, double goalY,
        const ConstraintSet& constraints, int alg = ALG_ASTAR,
        double timeLimit = 1.0, double w = 1.0);
    
    void preprocess(int pre = PRE_NONE, int clusters = 0, size_t seed = 12345);
    bool isPreprocessed() const;
    void preprocessClusters(int clusters);

    // this method used that edges of model are cylinders
    // and that manipulator has geom numbers 1 .. _dof inclusively
    // calculate length only 1 time
    double modelLength() const;
    // calculate answer only 1 time
    double maxActionLength() const;
    // return coords of site by state of joints
    std::pair<double, double> sitePosition(size_t armNum, const JointState& state) const;

    const int units = g_units;
    const double eps = g_eps;

    const vector<Action>& getPrimitiveActions() const;
    const Action& getZeroAction() const;

private:
    void initPrimitiveActions();

    JointState sampleFreeState(int attempts = 0);

    Solution linearPlanning(const JointState& startPos, const JointState& goalPos);

    Solution astarPlanning(
        size_t armNum, const JointState& startPos, const JointState& goalPos,
        const ConstraintSet& constraints,
        float weight, double timeLimit
    );
    Solution astarPlanning(
        size_t armNum, const JointState& startPos, double goalX, double goalY,
        const ConstraintSet& constraints,
        float weight, double timeLimit
    );

    Solution lazyAstarPlanning(
        size_t armNum, const JointState& startPos, const JointState& goalPos,
        const ConstraintSet& constraints,
        float weight, double timeLimit
    );
    Solution lazyAstarPlanning(
        size_t armNum, const JointState& startPos, double goalX, double goalY,
        const ConstraintSet& constraints,
        float weight, double timeLimit
    );

    Solution lazyARAstarPlanning(
        size_t armNum, const JointState& startPos, const JointState& goalPos,
        const ConstraintSet& constraints,
        float weight, double timeLimit
    );

    Solution preprocClustersPlanning(
        const JointState& startPos, const JointState& goalPos,
        float weight, double timeLimit
    );

    Solution preprocARAstarPlanning(
        const JointState& startPos, const JointState& goalPos,
        float weight, double timeLimit
    );

    vector<Action> _primitiveActions; // actions of one manipulator
    Action _zeroAction; // vector of zeroes
    size_t _dof; // dof of one manipulator
    size_t _arms; // the number of manipulators

    PreprocData _preprocData;

    mutable mjModel* _model; // model for collision checks
    mutable mjData* _data; // data for collision checks and calculations

    class AstarChecker : public astar::IAstarChecker
    {
    public:
        AstarChecker(ManipulatorPlanner* planner, size_t armNum, const JointState& goal, const ConstraintSet& constraints);

        bool isCorrect(const JointState& state, int stepNum, const Action& action) override;
        bool isGoal(const JointState& state, int stepNum) override;
        CostType costAction(const JointState& state, const Action& action) override;
        const std::vector<Action>& getActions() override;
        const Action& getZeroAction() override;
        CostType heuristic(const JointState& state) override;
    protected:
        ManipulatorPlanner* _planner;
        size_t _armNum;
        const JointState& _goal;
        ConstraintSet _constraints;
    };

    class AstarCheckerSite : public astar::IAstarChecker
    {
    public:
        AstarCheckerSite(ManipulatorPlanner* planner, size_t armNum, double goalX, double goalY, const ConstraintSet& constraints);

        bool isCorrect(const JointState& state, int stepNum, const Action& action) override;
        bool isGoal(const JointState& state, int stepNum) override;
        CostType costAction(const JointState& state, const Action& action) override;
        const std::vector<Action>& getActions() override;
        const Action& getZeroAction() override;
        CostType heuristic(const JointState& state) override;
    protected:
        ManipulatorPlanner* _planner;
        size_t _armNum;
        ConstraintSet _constraints;
        double _goalX;
        double _goalY;
    };
};
