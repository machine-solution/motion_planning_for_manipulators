#include "astar.h"
#include "utils.h"

#include <vector>

namespace astar {

SearchNode::SearchNode(CostType g, CostType h, CostType w, const JointState& state, int stepNum, int actionNum, SearchNode* parent, bool isLazy)
{
    _g = g;
    _h = h;
    _w = w;
    _state = state;
    _stepNum = stepNum;
    _actionNum = actionNum;
    _parent = parent;
    _isLazy = isLazy;
}

CostType SearchNode::g() const
{
    return _g;
}
CostType SearchNode::h() const
{
    return _h * _w;
}
CostType SearchNode::f() const
{
    return g() + h();
}
int SearchNode::actionNum() const
{
    return _actionNum;
}
int SearchNode::stepNum() const
{
    return _stepNum;
}
const JointState &SearchNode::state() const
{
    return _state;
}
SearchNode* SearchNode::parent()
{
    return _parent;
}

void SearchNode::updateLazy(bool newLazy)
{
    _isLazy = newLazy;
}
bool SearchNode::isLazy() const
{
    return _isLazy;
}

void SearchNode::updateWeight(CostType w)
{
    _w = w;
}

size_t SearchNode::byteSize() const
{
    return _state.byteSize() + sizeof(_g) * 3 + sizeof(_actionNum) + sizeof(_parent);
}

bool SearchNode::operator<(const SearchNode& sn)
{
    return f() == sn.f() ? -g() < -sn.g() : f() < sn.f();
}

bool CmpByState::operator()(SearchNode* a, SearchNode* b) const
{
    return std::pair<JointState, size_t>({a->state(), a->stepNum()}) < std::pair<JointState, size_t>({b->state(), b->stepNum()});
}
bool CmpByPriority::operator()(SearchNode* a, SearchNode* b) const
{
    return *a < *b;
}


SearchTree::SearchTree() {}
SearchTree::~SearchTree()
{
    while (!_open.empty())
    {
        SearchNode* node = *_open.begin();
        _open.erase(_open.begin());
        delete node;
    }

    while (!_closed.empty())
    {
        SearchNode* node = *_closed.begin();
        _closed.erase(_closed.begin());
        delete node;
    }
}

void SearchTree::addToOpen(SearchNode* node)
{
    startProfiling();
    _open.insert(node);
    stopProfiling();
}
void SearchTree::addToClosed(SearchNode* node)
{
    startProfiling();
    _closed.insert(node);
    stopProfiling();
}

SearchNode* SearchTree::extractBestNode()
{
    startProfiling();
    while (!_open.empty())
    {
        SearchNode* best = *_open.begin();
        _open.erase(_open.begin());
        if (wasExpanded(best))
        {
            // we must delete this node
            delete best;
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

size_t SearchTree::size() const
{
    return _open.size() + _closed.size();
}
size_t SearchTree::sizeOpen() const
{
    return _open.size();
}

bool SearchTree::wasExpanded(SearchNode* node) const
{
    startProfiling();
    bool res = _closed.count(node);
    stopProfiling();
    return res;
}

vector<SearchNode*> generateSuccessors(
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
        if (!checker.isCorrect(node->state(), node->stepNum(), action))
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
    SearchNode* startNode = new astar::SearchNode(0, checker.heuristic(startPos), weight, startPos, -1);
    tree.addToOpen(startNode);
    SearchNode* currentNode = tree.extractBestNode();

    // stats
    solution.stats.pathPotentialCost = checker.heuristic(startPos);

    while (currentNode != nullptr)
    {
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
        vector<astar::SearchNode*> successors = generateSuccessors(currentNode, checker, weight);
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
            solution.stats.consideredEdges++;
            solution.stats.evaluatedEdges++;
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
