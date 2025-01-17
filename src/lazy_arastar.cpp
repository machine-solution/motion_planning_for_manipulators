#include "astar.h"
#include "lazy_astar.h"
#include "lazy_arastar.h"

namespace astar {

SearchTreeARA::SearchTreeARA() {}
SearchTreeARA::~SearchTreeARA()
{
    clearOpen();
    clearClosed();
    clearHistory();
    clearIncons();
}

void SearchTreeARA::addToOpen(SearchNode* node)
{
    startProfiling();
    _open.insert(node);
    stopProfiling();
}
void SearchTreeARA::addToClosed(SearchNode* node)
{
    startProfiling();
    _closed.insert(node);
    stopProfiling();
}

void SearchTreeARA::addToIncons(SearchNode *node)
{
    startProfiling();
    auto iter = _incons.find(node);
    if (iter == _incons.end())
    {
        _incons.insert(node);
        stopProfiling();
        return;
    }
    SearchNode* best = *iter;
    if (best->g() > node->g())
    {
        _incons.erase(best);
        delete best;
        _incons.insert(node);
        stopProfiling();
        return;
    }
    delete node;
    stopProfiling();
    return;
}

void SearchTreeARA::clearOpen()
{
    while (!_open.empty())
    {
        SearchNode* node = *_open.begin();
        _open.erase(_open.begin());
        delete node;
    }
}

void SearchTreeARA::clearClosed()
{
    while (!_closed.empty())
    {
        SearchNode* node = *_closed.begin();
        _closed.erase(_closed.begin());
        _history.push_back(node);
    }
}

void SearchTreeARA::clearIncons()
{
    while (!_incons.empty())
    {
        SearchNode* node = *_incons.begin();
        _incons.erase(_incons.begin());
        delete node;
    }
}

void SearchTreeARA::clearHistory()
{
    while (!_history.empty())
    {
        SearchNode* node = _history.back();
        _history.pop_back();
        delete node;
    }
}

SearchNode* SearchTreeARA::extractBestNode()
{
    startProfiling();
    while (!_open.empty())
    {
        SearchNode* best = *_open.begin();
        _open.erase(_open.begin());
        if (wasExpanded(best))
        {
            // we must use it as incons
            addToIncons(best);
        }
        else
        {
            stopProfiling();
            return best;
        }
    }
    stopProfiling();
    return nullptr;
}

size_t SearchTreeARA::size() const
{
    return _open.size() + _closed.size();
}
size_t SearchTreeARA::sizeOpen() const
{
    return _open.size();
}

bool SearchTreeARA::wasExpanded(SearchNode* node) const
{
    startProfiling();
    bool res = _closed.count(node);
    stopProfiling();
    return res;
}

void SearchTreeARA::updateOpen(CostType w)
{
    startProfiling();
    multiset<SearchNode*, CmpByPriority> _newOpen;

    while (!_open.empty())
    {
        SearchNode* node = *_open.begin();
        _open.erase(_open.begin());
        node->updateWeight(w);
        _newOpen.insert(node);
    }
    while (!_incons.empty())
    {
        SearchNode* node = *_incons.begin();
        _incons.erase(_incons.begin());
        node->updateWeight(w);
        _newOpen.insert(node);
    }
    _open = _newOpen;
    clearClosed();
    stopProfiling();
}


Solution improveSolution(
    const JointState& startPos,
    IAstarChecker& checker,
    SearchTreeARA& tree,
    CostType goalF,
    double weight,
    double timeLimit
)
{
    Solution solution(checker.getActions(), checker.getZeroAction());
    clock_t clockTimeLimit = timeLimit * CLOCKS_PER_SEC;

    // start timer
    clock_t start = clock();

    SearchNode* currentNode = tree.extractBestNode();

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

        if (checker.isGoal(currentNode->state()))
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

        // if f() more than previous
        if (goalF <= currentNode->f())
        {
            tree.addToOpen(currentNode);
            solution.stats.pathVerdict = PATH_NOT_FOUND;
            break;
        }

        // expand current node
        vector<astar::SearchNode*> successors = lazyGenerateSuccessors(currentNode, checker, weight);
        for (auto successor : successors)
        {
            if (!tree.wasExpanded(successor))
            {
                tree.addToOpen(successor);
            }
            else
            {
                tree.addToIncons(successor);
            }
            solution.stats.consideredEdges++;
        }
        // add node to closed and incons
        tree.addToClosed(currentNode);
        SearchNode* copyNode = new SearchNode(*currentNode);
        tree.addToIncons(copyNode);

        // retake node from tree
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
        solution.stats.pathVerdict = PATH_NOT_FOUND;
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

Solution improveSolutionCycle(
    const JointState& startPos,
    IAstarChecker& checker,
    SearchTreeARA& tree,
    Solution solution, // copy value
    CostType goalF,
    double weight,
    double timeLimit
)
{
    clock_t clockTimeLimit = timeLimit * CLOCKS_PER_SEC;
    double currentWeight = weight;
    bool optimalFound = false;
    double mult = 0.9;

    // start timer
    clock_t start = clock();

    while (tree.sizeOpen() > 0)
    {
        if (optimalFound)
        {
            break;
        }
        if (currentWeight < 1.0)
        {
            currentWeight = 1.0;
            optimalFound = true;
        }
        clock_t middle = clock();
        double middle_runtime = (double)(middle - start) / CLOCKS_PER_SEC;
        if (middle - start > clockTimeLimit)
        {
            break;
        }
        Solution nextSolution = improveSolution(
            startPos,
            checker,
            tree,
            goalF,
            currentWeight,
            (timeLimit - middle_runtime)
        );
        if (nextSolution.stats.pathVerdict == PATH_FOUND)
        {
            solution = nextSolution;
            goalF = solution.stats.pathCost;
        }
        else if (nextSolution.stats.pathVerdict == PATH_NOT_FOUND)
        {
        }
        else
        {
            solution = nextSolution;
            break;
        }
        currentWeight *= mult;
        tree.updateOpen(currentWeight);
    }

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;
    return solution;
}

Solution lazyARAstar(
    const JointState &startPos,
    IAstarChecker &checker,
    double weight,
    double timeLimit)
{
    Solution solution(checker.getActions(), checker.getZeroAction());
    solution.stats.pathPotentialCost = checker.heuristic(startPos);

    // init search tree
    SearchTreeARA tree;
    SearchNode* startNode = new astar::SearchNode(0, checker.heuristic(startPos), weight, startPos, 0);
    tree.addToOpen(startNode);

    return improveSolutionCycle(
        startPos,
        checker,
        tree,
        solution,
        INFINITY,
        weight,
        timeLimit
    );
}

Solution lazyARAstar(const JointState &startPos, IAstarChecker &checker, Solution &startSolution, double weight, double timeLimit)
{
    // start timer
    clock_t start = clock();

    SearchTreeARA tree;
    SearchNode* startNode = new astar::SearchNode(0, checker.heuristic(startPos), weight, startPos, 0);
    tree.addToOpen(startNode);

    Solution solution(startSolution);
    solution.stats.pathPotentialCost = checker.heuristic(startPos);

    JointState currentState(startPos);
    CostType g = 0;
    size_t stepNum = 0;
    SearchNode* parent = nullptr;
    while (!startSolution.goalAchieved())
    {
        int stepId = startSolution.thisActionId();
        Action action = startSolution.nextAction();
        g += checker.costAction(currentState, action);
        currentState.applied(action);
        SearchNode* current = new SearchNode(
            g, checker.heuristic(currentState), weight, currentState, stepNum, stepId, parent, false
        );
        tree.addToOpen(
            current
        );
        parent = current;
        ++stepNum;
    }

    if (startSolution.stats.pathVerdict != PATH_FOUND)
    {
        g = INFINITY;
    }

    // end timer
    clock_t end = clock();

    return improveSolutionCycle(
        startPos,
        checker,
        tree,
        solution,
        g,
        weight,
        timeLimit - (double)(end - start) / CLOCKS_PER_SEC
    );
}

} // namespace astar
