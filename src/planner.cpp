#include "planner.h"
#include "utils.h"
#include "light_mujoco.h"

#include <time.h>

#include <stdio.h>

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveSteps();
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

bool ManipulatorPlanner::checkCollisionAction(const JointState& start, const JointState& delta) const
{
    startProfiling();
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = start.rad(i);
    }

    int jump = 8;
    for (size_t t = jump; t <= g_unitSize; t += jump)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i] = start.rad(i) + g_worldEps * delta[i] * t; // temporary we use global constant here for speed
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

vector<string> ManipulatorPlanner::configurationSpace() const
{
    vector<string> cSpace(g_units * 2, string(g_units * 2, '.'));
    for (int i = -g_units; i < g_units; ++i)
    {
        for (int j = -g_units; j < g_units; ++j)
        {
            if (checkCollision({i, j}))
                cSpace[g_units - 1 - j][i + g_units] = '@';
        }
    }
    return cSpace;
}

Solution ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& goalPos, int alg)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(goalPos))
    {
        return Solution(_primitiveSteps, _zeroStep); // incorrect aim
    }
    switch (alg)
    {
    case ALG_LINEAR:
        return linearPlanning(startPos, goalPos);
    case ALG_ASTAR:
        return astarPlanning(startPos, goalPos, manhattanHeuristic, 1.0);
    default:
        return Solution(_primitiveSteps, _zeroStep);
    }
}

void ManipulatorPlanner::initPrimitiveSteps()
{
    _zeroStep = JointState(_dof, 0);

    _primitiveSteps.assign(2 * _dof, JointState(_dof, 0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveSteps[i][i] = 1;
        _primitiveSteps[i + _dof][i] = -1;
    }
}

Solution ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    Solution solution(_primitiveSteps, _zeroStep);

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
                t = i + _dof;
            }

            if (checkCollisionAction(currentPos, _primitiveSteps[t]))
            {
                return solution; // we temporary need to give up : TODO
            }
            currentPos += _primitiveSteps[t];
            solution.addStep(t);
        }
    }

    solution.stats.pathFound = true;
    return solution;
}

CostType ManipulatorPlanner::costMove(const JointState& state1, const JointState& state2)
{
    return manhattanDistance(state1, state2);
}
vector<astar::SearchNode*> ManipulatorPlanner::generateSuccessors(
    astar::SearchNode* node,
    const JointState& goal,
    CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
    float weight
)
{
    startProfiling();
    vector<astar::SearchNode*> result;
    for (size_t i = 0; i < _primitiveSteps.size(); ++i)
    {
        JointState newState = node->state() + _primitiveSteps[i];
        if (checkCollisionAction(node->state(), _primitiveSteps[i]))
        {
            continue;
        }
        if (!newState.isCorrect())
        {
            continue;
        }
        result.push_back(
            new astar::SearchNode(
                node->g() + costMove(node->state(), newState),
                heuristicFunc(newState, goal) * weight,
                newState,
                i,
                node
            )
        );
    }

    stopProfiling();
    return result;
}

Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    CostType (*heuristicFunc)(const JointState& state1, const JointState& state2),
    float weight
)
{
    Solution solution(_primitiveSteps, _zeroStep);

    // start timer
    clock_t start = clock();

    // init search tree
    astar::SearchTree tree;
    astar::SearchNode* startNode = new astar::SearchNode(0, heuristicFunc(startPos, goalPos) * weight, startPos);
    tree.addToOpen(startNode);
    astar::SearchNode* currentNode = tree.extractBestNode();

    while (currentNode != nullptr)
    {
        if (currentNode->state() == goalPos)
        {
            break;
        }
        // count statistic
        solution.stats.maxTreeSize = std::max(solution.stats.maxTreeSize, tree.size());
        ++solution.stats.expansions;
        // expand current node
        vector<astar::SearchNode*> successors = generateSuccessors(currentNode, goalPos, heuristicFunc, weight);
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
        }
        // retake node from tree
        tree.addToClosed(currentNode);
        currentNode = tree.extractBestNode();
    }

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;

    if (currentNode != nullptr)
    {
        solution.stats.pathFound = true;
        solution.plannerProfile = getNamedProfileInfo();
        solution.searchTreeProfile = tree.getNamedProfileInfo();
        
        vector<size_t> steps;
        while (currentNode->parent() != nullptr)
        {
            // count stats
            solution.stats.pathCost += costMove(currentNode->state(), currentNode->parent()->state());
            //
            steps.push_back(currentNode->stepNum());
            currentNode = currentNode->parent();
        }

        // push steps
        for (int i = steps.size() - 1; i >= 0; --i)
        {
            solution.addStep(steps[i]);
        }
    }

    return solution;
}
