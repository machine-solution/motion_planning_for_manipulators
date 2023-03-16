#include "astar.h"

#include <vector>

namespace astar {

SearchNode::SearchNode(int g, int h, const JointState& state, int stepNum, SearchNode* parent)
{
    _g = g;
    _h = h;
    _f = _g + _h;
    _state = state;
    _stepNum = stepNum;
    _parent = parent;
}

int SearchNode::g() const
{
    return _g;
}
int SearchNode::h() const
{
    return _h;
}
int SearchNode::f() const
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
SearchNode* SearchNode::parent()
{
    return _parent;
}

bool SearchNode::operator<(const SearchNode& sn)
{
    return f() == sn.f() ? -g() < -sn.g() : f() < sn.f();
}

bool CmpByState::operator()(SearchNode* a, SearchNode* b) const
{
    return a->state() < b->state();
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

bool SearchTree::wasExpanded(SearchNode* node) const
{
    startProfiling();
    bool res = _closed.count(node);
    stopProfiling();
    return res;
}

} // namespace astar
