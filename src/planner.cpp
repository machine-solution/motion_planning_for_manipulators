#include "planner.h"
#include "utils.h"
#include "light_mujoco.h"

#include <time.h>

#include <stdio.h>
#include <queue>


PreprocData::PreprocData()
{
    isPreprocessed = false;
}

size_t PreprocData::byteSize() const
{
    size_t mapSize = actionsMap.size() * sizeof(size_t);
    if (!actionsMap.empty())
    {
        mapSize += actionsMap.size() * actionsMap.begin()->first.byteSize();
    }
    return mapSize + sizeof(isPreprocessed) + homeState.byteSize();
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

Solution ManipulatorPlanner::planActions(const JointState& startPos, const JointState& goalPos, int alg, double timeLimit, double w)
{
    clearAllProfiling(); // reset profiling

    if (true) // experiment TODO
    {
        preprocess();
        printf("DEBUG LOG: start finding path by preprocessed data\n");
        printf("DEBUG LOG: %zu KBytes used for preproc\n", _preprocData.kbyteSize());
        return preprocPlanning(startPos, goalPos);
    }

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
    default:
        return Solution(_primitiveActions, _zeroAction);
    }
}

Solution ManipulatorPlanner::planActions(const JointState& startPos, double goalX, double goalY, int alg, double timeLimit, double w)
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

void ManipulatorPlanner::preprocess()
{
    printf("DEBUG LOG: preprocessing started\n");
    startProfiling();
    if (isPreprocessed())
    {
        stopProfiling();
        return;
    }

    _preprocData.homeState = sampleFreeState(10);
    if (_preprocData.homeState.dof() != _dof)
    {
        stopProfiling();
        return;
    }

    std::queue<JointState> open;
    open.push(_preprocData.homeState);

    while (!open.empty())
    {
        JointState state = open.front();
        open.pop();
        for (int i = 0; i < _primitiveActions.size(); ++i)
        {
            JointState newState = state.applied(_primitiveActions[i]);
            if (
                newState.isCorrect()
                &&
                _preprocData.actionsMap.find(newState) == _preprocData.actionsMap.end()
                &&
                !checkCollisionAction(state, _primitiveActions[i])
            )
            {
                // reversed action
                _preprocData.actionsMap[newState] = _primitiveActions.size() - 1 - i;
                open.push(newState);
                // printf("Size of map: %zu of %f\n", _preprocData.actionsMap.size(), pow(2 * g_units, _dof));
            }
        }
    }
    printf("DEBUG LOG: preprocessing really comleted\n");
    
    _preprocData.isPreprocessed = true;
    stopProfiling();
}

bool ManipulatorPlanner::isPreprocessed() const
{
    return _preprocData.isPreprocessed;
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

Solution ManipulatorPlanner::preprocPlanning(const JointState& startPos, const JointState& goalPos)
{
    if (!isPreprocessed())
    {
        return Solution(_primitiveActions, _zeroAction);
    }

    // start timer
    clock_t start = clock();

    Solution startToHome(_primitiveActions, _zeroAction);
    JointState state = startPos;
    // TODO use other function
    // TODO infinity loop
    while (state != _preprocData.homeState)
    {
        size_t i = _preprocData.actionsMap[state];
        startToHome.addAction(_preprocData.actionsMap[state]);
        state.apply(_primitiveActions[i]);
    }
    Solution goalToHome(_primitiveActions, _zeroAction);
    state = goalPos;
    while (state != _preprocData.homeState)
    {
        size_t i = _preprocData.actionsMap[state];
        goalToHome.addAction(_preprocData.actionsMap[state]);
        state.apply(_primitiveActions[i]);
    }

    printf("DEBUG LOG: preprocessed solution found\n");

    Solution solution(_primitiveActions, _zeroAction);
    solution.add(startToHome);
    solution.add(goalToHome.reversed());

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;
    solution.stats.byteSize = _preprocData.byteSize();

    solution.stats.pathVerdict = PATH_FOUND;
    solution.plannerProfile = getNamedProfileInfo();
    for (size_t i = 0; i < solution.size(); ++i)
    {
        solution.stats.pathCost += solution[i].abs(); // cost action crutch
    }

    printf("DEBUG LOG: preprocessed solution's stats filled\n");
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
