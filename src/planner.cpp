#include "planner.h"
#include "utils.h"
#include "light_mujoco.h"

#include <time.h>

#include <stdio.h>
#include <queue>


Cluster::Cluster(const JointState &center)
{
    _center = center;
}

int Cluster::dist(const JointState &state) const
{
    return manhattanDistance(_center, state);
}

void Cluster::setCenter(const JointState &center)
{
    _center = center;
}

JointState Cluster::getCenter() const
{
    return _center;
}

void Cluster::setSolution(const Solution &solution)
{
    _solution = solution;
}

Solution Cluster::getSolution() const
{
    return _solution;
}

size_t Cluster::byteSize() const
{
    return _center.byteSize() + _solution.byteSize();
}

PreprocData::PreprocData()
{
    isPreprocessed = false;
}

size_t PreprocData::byteSize() const
{
    size_t clusterSize = 0;
    for (int i = 0; i < clusters.size(); ++i)
    {
        clusterSize += clusters[i].byteSize();
    }
    return clusterSize + sizeof(isPreprocessed) + sizeof(preprocRuntime) + homeState.byteSize();
}

size_t PreprocData::kbyteSize() const
{
    return byteSize() / (1024);
}

size_t PreprocData::mbyteSize() const
{
    return byteSize() / (1024 * 1024);
}

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveActions();
}

size_t ManipulatorPlanner::dof() const
{
    return _dof;
}

bool ManipulatorPlanner::checkCollision(const JointState& position) const
{
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position.rad(i);
    }
    return mj_light_collision(_model, _data);
}

bool ManipulatorPlanner::checkCollisionAction(const JointState& start, const Action& action) const
{
    startProfiling();
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        stopProfiling();
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = start.rad(i);
    }

    int jump = g_unitSize / g_checkJumps;
    for (size_t t = jump; t <= g_unitSize; t += jump)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i] = start.rad(i) + g_worldEps * action[i] * t; // temporary we use global constant here for speed
        }
        if (mj_light_collision(_model, _data))
        {
            stopProfiling();
            return true;
        }
    }
    stopProfiling();
    return false;
}

void putSymbol(vector<string>& cSpace, int i, int j, char symbol)
{
    cSpace[g_units - 1 - j][i + g_units] = symbol;
}

vector<string> ManipulatorPlanner::configurationSpace() const
{
    vector<string> cSpace(g_units * 2, string(g_units * 2, '.'));
    for (int i = -g_units; i < g_units; ++i)
    {
        for (int j = -g_units; j < g_units; ++j)
        {
            if (checkCollision({i, j}))
                putSymbol(cSpace, i, j, '@');
        }
    }
    return cSpace;
}

vector<string> ManipulatorPlanner::pathInConfigurationSpace(const JointState& start, Solution solution) const
{
    vector<string> cSpace = configurationSpace();
    JointState state = start;
    putSymbol(cSpace, state[0], state[1], 'A');
    while (!solution.goalAchieved())
    {
        state.apply(solution.nextAction());
        putSymbol(cSpace, state[0], state[1], '+');
    }
    putSymbol(cSpace, state[0], state[1], 'B');
    return cSpace;
}

Solution ManipulatorPlanner::planActions(
    const JointState& startPos, const JointState& goalPos,
    int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(startPos) || checkCollision(goalPos))
    {
        Solution solution(_primitiveActions, _zeroAction);
        solution.stats.pathVerdict = PATH_NOT_EXISTS; // incorrect aim
        return  solution;
    }
    switch (alg)
    {
    case ALG_LINEAR:
        return linearPlanning(startPos, goalPos);
    case ALG_ASTAR:
        return astarPlanning(startPos, goalPos, w, timeLimit);
    case ALG_LAZY_ASTAR:
        return lazyAstarPlanning(startPos, goalPos, w, timeLimit);
    case ALG_PREPROC_CLUSTERS:
        return preprocClustersPlanning(startPos, goalPos, w, timeLimit);
    case ALG_ARASTAR:
        return lazyARAstarPlanning(startPos, goalPos, w, timeLimit);
    case ALG_PREPROC_ARASTAR:
        return preprocARAstarPlanning(startPos, goalPos, w, timeLimit);
    default:
        return Solution(_primitiveActions, _zeroAction);
    }
}

Solution ManipulatorPlanner::planActions(
    const JointState& startPos, double goalX, double goalY,
    int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(startPos))
    {
        Solution solution(_primitiveActions, _zeroAction);
        solution.stats.pathVerdict = PATH_NOT_EXISTS; // incorrect aim
        return  solution;
    }
    switch (alg)
    {
    case ALG_ASTAR:
        return astarPlanning(startPos, goalX, goalY, w, timeLimit);
    case ALG_LAZY_ASTAR:
        return lazyAstarPlanning(startPos, goalX, goalY, w, timeLimit);
    default:
        return Solution(_primitiveActions, _zeroAction);
    }
}

void ManipulatorPlanner::preprocess(int pre, int clusters, size_t seed)
{
    srand(seed);
    switch (pre)
    {
    case PRE_NONE:
        return;
    case PRE_CLUSTERS:
        preprocessClusters(clusters);
        return;
    default:
        return;
    }
}

bool ManipulatorPlanner::isPreprocessed() const
{
    return _preprocData.isPreprocessed;
}

void ManipulatorPlanner::preprocessClusters(int clusters)
{
    startProfiling();
    if (isPreprocessed())
    {
        stopProfiling();
        return;
    }

    // start timer
    clock_t start = clock();

    _preprocData.homeState = sampleFreeState(10);
    if (_preprocData.homeState.dof() == 0)
    {
        stopProfiling();
        return;
    }

    // sample centers
    while (_preprocData.clusters.size() < clusters)
    {
        JointState center = sampleFreeState(10);
        if (center.dof() == 0)
        {
            continue;
        }
        Solution solution = planActions(
            center,
            _preprocData.homeState,
            ALG_LAZY_ASTAR,
            60.0,
            100.0
        );
        if (solution.stats.pathVerdict != PATH_FOUND)
        {
            continue;
        }
        _preprocData.clusters.push_back(
            Cluster(center)
        );
        _preprocData.clusters.back().setSolution(solution);
    }

    // end timer
    clock_t end = clock();
    _preprocData.preprocRuntime = (double)(end - start) / CLOCKS_PER_SEC;
    _preprocData.isPreprocessed = true;
    stopProfiling();
}

double ManipulatorPlanner::modelLength() const
{
    static double len = 0;
    if (len == 0)
    {
        for (size_t i = 1; i <= _dof; ++i)
        {
            len += _model->geom_size[i * 3 + 1] * 2;
        }
    }
    return len;
}
double ManipulatorPlanner::maxActionLength() const
{
    static double maxAction = sin(g_eps / 2) * modelLength() * 2;
    return maxAction;
}
std::pair<double, double> ManipulatorPlanner::sitePosition(const JointState& state) const
{
    startProfiling();
    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = state.rad(i);
    }
    mj_forward(_model, _data);
    stopProfiling();
    return {_data->site_xpos[0], _data->site_xpos[1]};
}

const vector<Action>& ManipulatorPlanner::getPrimitiveActions() const
{
    return _primitiveActions;
}

// opposite actions must be reversed
void ManipulatorPlanner::initPrimitiveActions()
{
    _zeroAction = Action(_dof, 0);

    _primitiveActions.assign(2 * _dof, Action(_dof, 0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveActions[i][i] = 1;
        _primitiveActions[2 * _dof - i - 1][i] = -1;
    }
}

JointState ManipulatorPlanner::sampleFreeState(int attempts)
{
    JointState state = randomState(_dof);
    while (attempts-- > 0 && checkCollision(state))
    {
        state = randomState(_dof);
    }
    if (attempts < 0)
    {
        return JointState(0, 0);
    }
    else
    {
        return state;
    }
}

Solution ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    Solution solution(_primitiveActions, _zeroAction);

    JointState currentPos = startPos;
    for (size_t i = 0; i < _dof; ++i)
    {
        while (currentPos[i] != goalPos[i])
        {
            size_t t = -1;
            if (currentPos[i] < goalPos[i]) // + eps
            {
                t = i;
            }
            else if (currentPos[i] > goalPos[i]) // - eps
            {
                t = 2 * _dof - i - 1;
            }

            if (checkCollisionAction(currentPos, _primitiveActions[t]))
            {
                solution.stats.pathVerdict = PATH_NOT_FOUND;
                return solution; // we temporary need to give up : TODO
            }
            currentPos.apply(_primitiveActions[t]);
            solution.addAction(t);
        }
    }

    solution.stats.pathVerdict = PATH_FOUND;
    return solution;
}

Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    float weight, double timeLimit
)
{
    AstarChecker checker(this, goalPos);
    Solution solution = astar::astar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}
Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, double goalX, double goalY,
    float weight, double timeLimit
)
{
    AstarCheckerSite checker(this, goalX, goalY);
    Solution solution = astar::astar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}

Solution ManipulatorPlanner::lazyAstarPlanning(
    const JointState& startPos, const JointState& goalPos,
    float weight, double timeLimit
)
{
    AstarChecker checker(this, goalPos);
    Solution solution = astar::lazyAstar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}
Solution ManipulatorPlanner::lazyAstarPlanning(
    const JointState& startPos, double goalX, double goalY,
    float weight, double timeLimit
)
{
    AstarCheckerSite checker(this, goalX, goalY);
    Solution solution = astar::lazyAstar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}

Solution ManipulatorPlanner::lazyARAstarPlanning(
    const JointState& startPos, const JointState& goalPos,
    float weight, double timeLimit
)
{
    AstarChecker checker(this, goalPos);
    Solution solution = astar::lazyARAstar(startPos, checker, weight, timeLimit);
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}

Solution ManipulatorPlanner::preprocClustersPlanning(
    const JointState& startPos, const JointState& goalPos,
    float weight, double timeLimit
)
{
    if (_preprocData.clusters.size() == 0)
    {
        return Solution(_primitiveActions, _zeroAction);
    }

    // start timer
    clock_t start = clock();

    size_t startIdx = 0;
    int startDistance = INT32_MAX;
    size_t goalIdx = 0;
    int goalDistance = INT32_MAX;
    for (int i = 0; i < _preprocData.clusters.size(); ++i)
    {
        int newStartDst = _preprocData.clusters[i].dist(startPos);
        int newGoalDst  = _preprocData.clusters[i].dist(goalPos);

        if (newStartDst < startDistance)
        {
            startIdx = i;
            startDistance = newStartDst;
        }
        if (newGoalDst < goalDistance)
        {
            goalIdx = i;
            goalDistance = newGoalDst;
        }
    }

    clock_t middle = clock();
    double middle_runtime = (double)(middle - start) / CLOCKS_PER_SEC;

    Solution startToCluster = planActions(
        startPos, _preprocData.clusters[startIdx].getCenter(),
        ALG_LAZY_ASTAR, (timeLimit - middle_runtime), weight 
    );
    if (startToCluster.stats.pathVerdict != PATH_FOUND)
    {
        clock_t end = clock();
        startToCluster.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;
        startToCluster.stats.pathVerdict = PATH_NOT_FOUND;
        return startToCluster;
    }
    middle = clock();
    middle_runtime = (double)(middle - start) / CLOCKS_PER_SEC;

    Solution goalToCluster = planActions(
        goalPos, _preprocData.clusters[goalIdx].getCenter(),
        ALG_LAZY_ASTAR, (timeLimit - middle_runtime), weight 
    );
    if (goalToCluster.stats.pathVerdict != PATH_FOUND)
    {
        clock_t end = clock();
        goalToCluster.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;
        goalToCluster.stats.byteSize = std::max(
            goalToCluster.stats.byteSize,
            startToCluster.stats.byteSize
        );
        goalToCluster.stats.pathVerdict = PATH_NOT_FOUND;
        return goalToCluster;
    }

    startToCluster.add(_preprocData.clusters[startIdx].getSolution());
    startToCluster.add(_preprocData.clusters[goalIdx].getSolution().reversed());
    startToCluster.add(goalToCluster.reversed());

    // end timer
    clock_t end = clock();
    startToCluster.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;
    startToCluster.stats.pathVerdict = PATH_FOUND;
    startToCluster.stats.preprocByteSize = _preprocData.byteSize();
    startToCluster.stats.preprocRuntime = _preprocData.preprocRuntime;

    return startToCluster;
}

Solution ManipulatorPlanner::preprocARAstarPlanning(const JointState &startPos, const JointState &goalPos, float weight, double timeLimit)
{
    const float PREPROC_WEIGHT = 1000000.0;
    // start timer
    clock_t start = clock();

    printf("DEBUG LOG. dof = %zu, tl = %f\n", startPos.dof(), timeLimit);

    double preprocTimeLimit = std::min(timeLimit * 0.5, 2.5);
    printf("DEBUG LOG. For preprocess wew use %fs time limit\n", preprocTimeLimit);

    Solution startSolution = preprocClustersPlanning(
        startPos, goalPos, PREPROC_WEIGHT, preprocTimeLimit
    );

    // end timer
    clock_t end = clock();

    printf("DEBUG LOG. During finding preproc solution we spent %f seconds\n", (double)(end - start) / CLOCKS_PER_SEC);

    AstarChecker checker(this, goalPos);
    Solution solution = astar::lazyARAstar(
        startPos,
        checker,
        startSolution,
        weight,
        timeLimit - (double)(end - start) / CLOCKS_PER_SEC
    );
    solution.plannerProfile = getNamedProfileInfo();
    return solution;
}

// Checkers

ManipulatorPlanner::AstarChecker::AstarChecker(ManipulatorPlanner* planner, const JointState& goal) : _goal(goal)
{
    _planner = planner;
}

bool ManipulatorPlanner::AstarChecker::isCorrect(const JointState& state, const Action& action)
{
    return state.applied(action).isCorrect() && (!_planner->checkCollisionAction(state, action));
}
bool ManipulatorPlanner::AstarChecker::isGoal(const JointState& state)
{
    return state == _goal;
}
CostType ManipulatorPlanner::AstarChecker::costAction(const JointState& state, const Action& action)
{
    if (state.lastAction() == nullptr)
    {
        return action.abs();
    }
    else
    {
        return action.abs() + g_weightSmoothness * manhattanDistance(action, *state.lastAction());
    }
}
const std::vector<Action>& ManipulatorPlanner::AstarChecker::getActions()
{
    return _planner->_primitiveActions;
}
const Action& ManipulatorPlanner::AstarChecker::getZeroAction()
{
    return _planner->_zeroAction;
}
CostType ManipulatorPlanner::AstarChecker::heuristic(const JointState& state)
{
    return manhattanHeuristic(state, _goal);
}

// checker for site goal


ManipulatorPlanner::AstarCheckerSite::AstarCheckerSite(ManipulatorPlanner* planner, double goalX, double goalY)
{
    _planner = planner;
    _goalX = goalX;
    _goalY = goalY;
}

bool ManipulatorPlanner::AstarCheckerSite::isCorrect(const JointState& state, const Action& action)
{
    return state.applied(action).isCorrect() && (!_planner->checkCollisionAction(state, action));
}
bool ManipulatorPlanner::AstarCheckerSite::isGoal(const JointState& state)
{
    const double r = 0.05; // const minimum dist from pos
    if (state.hasCacheXY())
    {
        double dx = state.cacheX() - _goalX;
        double dy = state.cacheY() - _goalY;
        return dx * dx + dy * dy <= r * r;
    }
    else
    {
        std::pair<double, double> xy = _planner->sitePosition(state);
        state.setCacheXY(xy.first, xy.second);
        double dx = xy.first - _goalX;
        double dy = xy.second - _goalY;
        return dx * dx + dy * dy <= r * r;
    }
}
CostType ManipulatorPlanner::AstarCheckerSite::costAction(const JointState& state, const Action& action)
{
    if (state.lastAction() == nullptr)
    {
        return action.abs();
    }
    else
    {
        return action.abs() + g_weightSmoothness * manhattanDistance(action, *state.lastAction());
    }
}
const std::vector<Action>& ManipulatorPlanner::AstarCheckerSite::getActions()
{
    return _planner->_primitiveActions;
}
const Action& ManipulatorPlanner::AstarCheckerSite::getZeroAction()
{
    return _planner->_zeroAction;
}
CostType ManipulatorPlanner::AstarCheckerSite::heuristic(const JointState& state)
{
    if (state.hasCacheXY())
    {
        double dx = state.cacheX() - _goalX;
        double dy = state.cacheY() - _goalY;
        return sqrt(dx * dx + dy * dy) / _planner->maxActionLength();
    }
    else
    {
        std::pair<double, double> xy = _planner->sitePosition(state);
        state.setCacheXY(xy.first, xy.second);
        double dx = xy.first - _goalX;
        double dy = xy.second - _goalY;
        return sqrt(dx * dx + dy * dy) / _planner->maxActionLength();
    }
}
