#include "planner.h"

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveSteps();
}

JointState& ManipulatorPlanner::nextStep()
{
    if (_nextStepId >= _solveSteps.size())
    {
        return _zeroStep;
    }
    return _primitiveSteps[_solveSteps[_nextStepId++]];
}

bool ManipulatorPlanner::goalAchieved()
{
    return _nextStepId >= _solveSteps.size();
}

bool ManipulatorPlanner::checkCollision(const JointState& position)
{
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position.rad(i);
    }
    mj_step1(_model, _data);
    return _data->ncon;
}

void ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& goalPos, int alg)
{
    if (checkCollision(goalPos))
    {
        return; // incorrect aim
    }
    switch (alg)
    {
    case ALG_LINEAR:
        linearPlanning(startPos, goalPos);
        break;
    case ALG_ASTAR:
        astarPlanning(startPos, goalPos, manhattanDistance);
        break;
    default:
        // TODO exception
        break;
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

void ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    _nextStepId = 0;
    _solveSteps.clear();

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
                return; // we temporary need to give up : TODO
            }
            _solveSteps.push_back(t);
        }
    }
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
    vector<astar::SearchNode*> result;
    for (size_t i = 0; i < _primitiveSteps.size(); ++i)
    {
        JointState newState = node->state() + _primitiveSteps[i];
        if (checkCollision(newState))
        {
            continue;
        }
        if (newState.maxJoint() >= newState.units ||
            newState.minJoint() < -newState.units)
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
    return result;
}

void ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    int (*heuristicFunc)(const JointState& state1, const JointState& state2)
)
{
    // reset solve
    _nextStepId = 0;
    _solveSteps.clear();

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

    if (currentNode != nullptr)
    {
        vector<size_t> steps;
        while (currentNode->parent() != nullptr)
        {
            steps.push_back(currentNode->stepNum());
            currentNode = currentNode->parent();
        }

        // push steps
        for (int i = steps.size() - 1; i >= 0; --i)
        {
            _solveSteps.push_back(steps[i]);
        }
    }
    else
    {
        return; // give up
    }
}
