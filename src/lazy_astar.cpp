#include "astar.h"
#include "lazy_astar.h"

namespace astar {

vector<SearchNode*> lazyGenerateSuccessors(
    SearchNode* node,
    IAstarChecker& checker,
    double weight
)
{
    vector<SearchNode*> result;
    for (size_t i = 0; i < checker.getActions().size(); ++i)
    {
        Action action = checker.getActions()[i];
        JointState newState = node->state().applied(action);
        if (!newState.isCorrect())
        {
            continue;
        }
        result.push_back(
            new SearchNode(
                node->g() + checker.costAction(node->state(), action),
                checker.heuristic(newState),
                weight,
                newState,
                node->stepNum() + 1,
                i,
                node,
                true
            )
        );
    }

    return result;
}

Solution lazyAstar(
    const JointState& startPos,
    IAstarChecker& checker,
    double weight,
    double timeLimit
)
{
    Solution solution(checker.getActions(), checker.getZeroAction());
    clock_t clockTimeLimit = timeLimit * CLOCKS_PER_SEC;

    // start timer
    clock_t start = clock();

    // init search tree
    SearchTree tree;
    SearchNode* startNode = new astar::SearchNode(0, checker.heuristic(startPos), weight, startPos, -1);
    tree.addToOpen(startNode);
    SearchNode* currentNode = tree.extractBestNode();

    // stats
    solution.stats.pathPotentialCost = checker.heuristic(startPos);

    while (currentNode != nullptr)
    {
        if (currentNode->isLazy())
        {
            solution.stats.evaluatedEdges++;
            const Action& lastAction = checker.getActions()[currentNode->actionNum()];
            if (!checker.isCorrect(currentNode->parent()->state(), currentNode->stepNum(), lastAction))
            {
                delete currentNode;
                currentNode = tree.extractBestNode();
                continue;
            }
            else
            {
                // already not lazy
                currentNode->updateLazy(false);
            }
        }

        if (checker.isGoal(currentNode->state(), currentNode->stepNum()))
        {
            solution.stats.pathVerdict = PATH_FOUND;
            break;
        }
        // give up if time limit is exhausted
        if (clock() - start > clockTimeLimit)
        {
            solution.stats.pathVerdict = PATH_NOT_FOUND;
            break;
        }
        // expand current node
        vector<astar::SearchNode*> successors = lazyGenerateSuccessors(currentNode, checker, weight);
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
            solution.stats.consideredEdges++;
        }
        // retake node from tree
        tree.addToClosed(currentNode);
        currentNode = tree.extractBestNode();
        // count statistic
        solution.stats.byteSize = std::max(solution.stats.byteSize, tree.size());
        ++solution.stats.expansions;
    }

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;

    if (currentNode == nullptr)
    {
        solution.stats.pathVerdict = PATH_NOT_EXISTS;
    }
    else if (solution.stats.pathVerdict == PATH_FOUND)
    {
        solution.stats.byteSize *= currentNode->byteSize(); // max tree size * node bytes
        solution.stats.pathCost = currentNode->g();
        vector<size_t> actions;
        while (currentNode->parent() != nullptr)
        {
            actions.push_back(currentNode->actionNum());
            currentNode = currentNode->parent();
        }
        solution.stats.pathPotentialCost = checker.heuristic(startPos);

        // push actions
        for (int i = actions.size() - 1; i >= 0; --i)
        {
            solution.addAction(actions[i]);
        }
    }

    solution.searchTreeProfile = tree.getNamedProfileInfo();
    return solution;
}

} // namespace astar
