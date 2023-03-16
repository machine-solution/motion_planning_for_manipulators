#include "planner.h"

#include <time.h> 

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveSteps();
}

bool ManipulatorPlanner::checkCollision(const JointState& position)
{
    startProfiling();
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position.rad(i);
    }
    mj_step1(_model, _data);
    stopProfiling();
    return _data->ncon;
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
        return astarPlanning(startPos, goalPos, manhattanDistance);
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

            currentPos += _primitiveSteps[t];
            if (checkCollision(currentPos))
            {
                return solution; // we temporary need to give up : TODO
            }
            solution.addStep(t);
        }
    }

    solution.stats.pathFound = true;
    return solution;
}

int ManipulatorPlanner::costMove(const JointState& state1, const JointState& state2)
{
    return manhattanDistance(state1, state2);
}
vector<astar::SearchNode*> ManipulatorPlanner::generateSuccessors(
    astar::SearchNode* node,
    const JointState& goal,
    int (*heuristicFunc)(const JointState& state1, const JointState& state2)
)
{
    startProfiling();
    vector<astar::SearchNode*> result;
    for (size_t i = 0; i < _primitiveSteps.size(); ++i)
    {
        JointState newState = node->state() + _primitiveSteps[i];
        if (checkCollision(newState))
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
                heuristicFunc(newState, goal),
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
    int (*heuristicFunc)(const JointState& state1, const JointState& state2)
)
{
    Solution solution(_primitiveSteps, _zeroStep);

    // start timer
    clock_t start = clock();

    // init search tree
    astar::SearchTree tree;
    astar::SearchNode* startNode = new astar::SearchNode(0, heuristicFunc(startPos, goalPos), startPos);
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
        vector<astar::SearchNode*> successors = generateSuccessors(currentNode, goalPos, heuristicFunc);
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
        solution.plannerProfile = getSortedProfileInfo();
        solution.searchTreeProfile = tree.getSortedProfileInfo();
        
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
