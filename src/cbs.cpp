#include "cbs.h"
#include <iostream>

vector<CBSNode*> generateSuccessorsCBS(CBSNode* node, ManipulatorPlanner* planner, const MultiState& startPos, size_t constraintInterval)
{
    vector<CBSNode*> result;
    Conflict conflict = planner->findFirstConflict(startPos, node->solution());
    if (!conflict.has())
    {
        return vector<CBSNode*>();
    }

    int stepFrom = conflict.stepNum() - constraintInterval / 2;
    int stepTo = stepFrom + constraintInterval;

    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<VertexConstraint>(conflict.stepNum(), conflict.stepNum() + 1, conflict.firstState()),
            conflict.firstArm(),
            node,
            true
        )
    );
    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<VertexConstraint>(conflict.stepNum(), conflict.stepNum() + 1, conflict.secondState()),
            conflict.secondArm(),
            node,
            true
        )
    );

    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<AvoidanceConstraint>(stepFrom, stepTo, conflict.secondArm(), conflict.secondState()),
            conflict.firstArm(),
            node,
            true
        )
    );
    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<AvoidanceConstraint>(stepFrom, stepTo, conflict.firstArm(), conflict.firstState()),
            conflict.secondArm(),
            node,
            true
        )
    );

    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<SphereConstraint>(
                stepFrom, stepTo,
                conflict.point()[0],
                conflict.point()[1],
                conflict.point()[2]
            ),
            conflict.firstArm(),
            node,
            true
        )
    );
    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<SphereConstraint>(
                stepFrom, stepTo,
                conflict.point()[0],
                conflict.point()[1],
                conflict.point()[2]
            ),
            conflict.secondArm(),
            node,
            true
        )
    );

    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<PriorityConstraint>(
                conflict.secondArm()
            ),
            conflict.firstArm(),
            node,
            true
        )
    );
    result.push_back(
        new CBSNode(
            node->conflictCount(),
            node->sumG(),
            node->constraintsMap(),
            node->solution(),
            std::make_shared<PriorityConstraint>(
                conflict.firstArm()
            ),
            conflict.secondArm(),
            node,
            true
        )
    );

    return result;
}

std::vector<std::vector<size_t>> makePriorityGraph(size_t arms, CBSNode *node)
{
    std::vector<std::vector<size_t>> graph(arms);
    for (size_t a = 0; a < arms; ++a)
    {
        for (auto cst : node->constraints(a))
        {
            if (cst->type() == CONSTRAINT_PRIORITY)
            {
                auto priorityCst = std::static_pointer_cast<PriorityConstraint>(cst);
                graph[a].push_back(priorityCst->armNum());
            }
        }
    }
    return graph;
}

bool _threePhaseDfs(size_t v, std::vector<size_t>& used, const std::vector<std::vector<size_t>>& graph)
{
    used[v] = 1;
    for (size_t u : graph[v])
    {
        if (used[u] == 0)
        {
            bool ok = _threePhaseDfs(u, used, graph);
            if (!ok)
            {
                return false;
            }
        }
        else if (used[u] == 1)
        {
            return false;
        }
    }
    used[v] = 2;
    return true;
}

bool graphHasCycle(size_t arms, const std::vector<std::vector<size_t>>& graph)
{
    std::vector<size_t> used(arms, 0);
    for (size_t a = 0; a < arms; ++a)
    {
        if (used[a])
        {
            continue;
        }
        bool ok = _threePhaseDfs(a, used, graph);
        if (!ok)
        {
            return true;
        }
    }
    return false;
}


void _topsortDfs(size_t v, std::vector<size_t>& used, const std::vector<std::vector<size_t>>& graph, std::vector<size_t>& order)
{
    used[v] = 1;
    for (size_t u : graph[v])
    {
        if (!used[u])
        {
            _topsortDfs(u, used, graph, order);
        }
    }
    order.push_back(v);
}


// returns list of vertexes of graph from which startVertex is reachable in top sort order (startVertex is the last one for example)
std::vector<size_t> topSortReverse(size_t arms, const std::vector<std::vector<size_t>>& graph, size_t startVertex)
{
    std::vector<std::vector<size_t>> rGraph(arms);
    std::vector<size_t> used(arms, 0);
    std::vector<size_t> order;
    for (size_t a = 0; a < arms; ++a)
    {
        for (size_t b : graph[a])
        {
            rGraph[b].push_back(a);
        }
    }
    _topsortDfs(startVertex, used, rGraph, order);
    // order is a reverse topsort order for rGraph, but normal topsort order for graph
    return order;
}

bool evaluateNode(CBSNode *node, ManipulatorPlanner *planner, const MultiState &startPos, const MultiState &goalPos, double weight, double time)
{
    MultiSolution solution = node->solution();

    size_t oldConflictsCount = node->conflictCount();

    size_t arms = startPos.arms();

    std::vector<std::vector<size_t>> graph = makePriorityGraph(arms, node);
    if (graphHasCycle(arms, graph))
    {
        // make very big (infinite) loss function
        // to not consider this node
        node->updateLazy(
            (size_t)1e9,
            (CostType)1e9,
            solution
        );
        return false;
    }

    std::vector<size_t> order = topSortReverse(arms, graph, node->armNum());

    for (size_t armNum : order)
    {
        Solution armSolution = planner->planActions(
            armNum,
            startPos[armNum],
            goalPos[armNum],
            ConstraintSet(
                startPos,
                solution,
                node->constraints(armNum)
            ),
            ALG_LAZY_ASTAR,
            time,
            weight
        );
        solution[armNum] = armSolution;
    }

    size_t conflictsCount = planner->calculateConflictsCount(startPos, solution);
    CostType sumG = solution.countActions();
    node->updateLazy(
        conflictsCount,
        sumG,
        solution
    );

    return conflictsCount < oldConflictsCount;
}

bool _solutionIsCorrect(ManipulatorPlanner* planner, MultiSolution solution, MultiState startPos, const MultiState &goalPos, bool needZeroConflicts)
{
    bool pathFound = (planner->calculateConflictsCount(startPos, solution) == 0) || (!needZeroConflicts);
    for (size_t a = 0; a < planner->arms(); ++a)
    {
        pathFound &= (solution[a].stats.pathVerdict == PATH_FOUND);
    }
    while (!solution.goalAchieved())
    {
        startPos.apply(solution.nextAction());
    }
    pathFound &= (startPos == goalPos);
    return pathFound;
}

MultiSolution CBS(
    size_t dof, size_t arms, ManipulatorPlanner* planner, const MultiState &startPos, const MultiState &goalPos,
    double weight, double timeLimit, size_t constraintInterval
)
{
    clock_t clockTimeLimit = timeLimit * CLOCKS_PER_SEC;

    // start timer
    clock_t start = clock();

    // init solution
    MultiSolution solution(
        planner->getPrimitiveActions(), planner->getZeroAction(),
        dof, arms
    );
    for (size_t armNum = 0; armNum < arms; ++armNum)
    {
        Solution armSolution = planner->planActions(
            armNum,
            startPos[armNum], goalPos[armNum],
            ConstraintSet(
                startPos,
                solution,
                {}
            ),
            ALG_LAZY_ASTAR,
            (clockTimeLimit - clock() + start) / CLOCKS_PER_SEC,
            weight
        );
        solution[armNum] = armSolution;
    }
    if (planner->calculateConflictsCount(startPos, solution) == 0)
    {
        solution.stats.pathTrivial = 1;
    }
    // init search tree
    CBSTree tree(weight);
    CBSNode* startNode = new CBSNode(
        planner->calculateConflictsCount(startPos, solution),
        solution.countActions(),
        std::vector<ConstraintList>(arms),
        solution,
        std::make_shared<EmptyConstraint>(),
        0,
        nullptr,
        false
    );
    tree.addToOpen(startNode);
    CBSNode* currentNode = tree.extractBestNode();
    solution.stats.pathTrivialCost = currentNode->sumG();

    // stats
    // solution.stats.pathPotentialCost = checker.heuristic(startPos);

    while (currentNode != nullptr)
    {
        // give up if time limit is exhausted
        if (clock() - start > clockTimeLimit)
        {
            solution.stats.pathVerdict = PATH_NOT_FOUND;
            break;
        }
        if (currentNode->isLazy())
        {
            bool successEvaluate = evaluateNode(
                currentNode,
                planner,
                startPos,
                goalPos,
                weight,
                (clockTimeLimit - clock() + start) / CLOCKS_PER_SEC
            );
            bool pathIsCorrect = _solutionIsCorrect(planner, currentNode->solution(), startPos, goalPos, false);
            if (successEvaluate && pathIsCorrect)
            {
                tree.payReward(currentNode->newConstraintType());
            }
            else
            {
                tree.payPenalty(currentNode->newConstraintType());
            }
            if (pathIsCorrect)
            {
                tree.addToOpen(currentNode);
            }
            currentNode = tree.extractBestNode();
            continue;
        }
        bool pathFound = _solutionIsCorrect(planner, currentNode->solution(), startPos, goalPos, true);
        if (pathFound)
        {
            Stats stats = solution.stats;
            solution = currentNode->solution();
            solution.stats = stats;
            solution.stats.pathVerdict = PATH_FOUND;
            break;
        }
        // expand current node
        vector<CBSNode*> successors = generateSuccessorsCBS(
            currentNode,
            planner,
            startPos,
            constraintInterval
        );
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
            // solution.stats.consideredEdges++;
            // solution.stats.evaluatedEdges++;
        }
        // retake node from tree
        tree.addToClosed(currentNode);
        currentNode = tree.extractBestNode();
        // count statistic
        // solution.stats.byteSize = std::max(solution.stats.byteSize, tree.size());
        // ++solution.stats.expansions;
    }

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;

    if (currentNode == nullptr)
    {
        solution.stats.pathVerdict = PATH_NOT_FOUND;
    }
    else if (solution.stats.pathVerdict == PATH_FOUND)
    {
        // solution.stats.byteSize *= currentNode->byteSize(); // max tree size * node bytes
        solution.stats.pathCost = currentNode->sumG();
        
        // solution.stats.pathPotentialCost = checker.heuristic(startPos);

        Stats stats = solution.stats;
        // push actions
        solution = currentNode->solution();
        solution.reset();
        solution.stats = stats;
    }

    // solution.searchTreeProfile = tree.getNamedProfileInfo();
    solution.reset();
    return solution;
}
