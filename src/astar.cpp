#include "astar.h"
#include "utils.h"

#include <vector>
namespace astar {

SearchNode::SearchNode(CostType g, CostType h, const JointState& state, int stepNum, std::unique_ptr<SearchNode> parent)
{
    _g = g;
    _h = h;
    _f = _g + _h;
    _state = state;
    _stepNum = stepNum;
    auto _parent = std::move(parent);
}

CostType SearchNode::g() const
{
    return _g;
}
CostType SearchNode::h() const
{
    return _h;
}
CostType SearchNode::f() const
{
    return _f;
}
int SearchNode::stepNum() const
{
    return _stepNum;
}
const JointState& SearchNode::state() const
{
    return _state;
}
std::unique_ptr<SearchNode> SearchNode::parent()
{
    return std::move(_parent);
}

bool SearchNode::operator<(const SearchNode& sn)
{
    return f() == sn.f() ? -g() < -sn.g() : f() < sn.f();
}

bool CmpByState::operator()(std::unique_ptr<SearchNode> a, std::unique_ptr<SearchNode> b) const
{
    return a->state() < b->state();
}
bool CmpByPriority::operator()(std::unique_ptr<SearchNode> a, std::unique_ptr<SearchNode> b) const
{
    return *a < *b;
}


SearchTree::SearchTree() {}
SearchTree::~SearchTree()
{
    while (!_open.empty())
    {
        _open.erase(_open.begin());
    }

    while (!_closed.empty())
    {
        _closed.erase(_closed.begin());
    }
}

void SearchTree::addToOpen(std::unique_ptr<SearchNode> node)
{
    startProfiling();
    _open.insert(node);
    stopProfiling();
}
void SearchTree::addToClosed(std::unique_ptr<SearchNode> node)
{
    startProfiling();
    _closed.insert(node);
    stopProfiling();
}

std::unique_ptr<SearchNode> SearchTree::extractBestNode()
{
    startProfiling();
    while (!_open.empty())
    {
        auto best = std::move(*_open.begin());
        _open.erase(_open.begin());
        if (wasExpanded(best))
        {
            // we must delete this node
            delete best;
        }
        else
        {
            stopProfiling();
            return std::move(best);
        }
    }
    stopProfiling();
    return nullptr;
}

size_t SearchTree::size() const
{
    return _open.size() + _closed.size();
}
size_t SearchTree::sizeOpen() const
{
    return _open.size();
}


bool SearchTree::wasExpanded(std::unique_ptr<SearchNode> node) const
{
    startProfiling();
    bool res = _closed.count(node);
    stopProfiling();
    return res;
}

vector<std::unique_ptr<SearchNode>> generateSuccessors(
    std::unique_ptr<SearchNode> node,
    IAstarChecker& checker,
    double weight
)
{
    vector<std::unique_ptr<SearchNode>> result;
    for (size_t i = 0; i < checker.getActions().size(); ++i)
    {
        Action action = checker.getActions()[i];
        JointState newState = node->state().applied(action);
        if (!checker.isCorrect(node->state(), action))
        {
            continue;
        }
        result.push_back(
            new SearchNode(
                node->g() + checker.costAction(node->state(), action),
                checker.heuristic(newState) * weight,
                newState,
                i,
                node
            )
        );
    }

    return result;
}

Solution astar(
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
    std::unique_ptr<SearchNode> startNode = new astar::SearchNode(0, checker.heuristic(startPos) * weight, startPos);
    tree.addToOpen(startNode);
    std::unique_ptr<SearchNode> currentNode(tree.extractBestNode());

    while (currentNode != nullptr)
    {
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
        // expand current node
        vector<astar::std::unique_ptr<SearchNode>> successors = generateSuccessors(currentNode, checker, weight);
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
        }
        // retake node from tree
        tree.addToClosed(currentNode);
        currentNode = tree.extractBestNode();
        // count statistic
        solution.stats.maxTreeSize = std::max(solution.stats.maxTreeSize, tree.size());
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
        solution.stats.pathCost = currentNode->g();
        vector<size_t> actions;
        while (currentNode->parent() != nullptr)
        {
            actions.push_back(currentNode->stepNum());
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
