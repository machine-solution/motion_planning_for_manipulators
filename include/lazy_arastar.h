#pragma once

#include "joint_state.h"
#include "utils.h"
#include "astar.h"
#include "solution.h"

#include <set>
#include <map>

using std::set;
using std::map;
using std::multiset;

namespace astar
{

/*
This is a container and data structure for A* algorithm.
A* relies on this Tree in deletation nodes.
*/
class SearchTreeARA : public Profiler
{
public:
    SearchTreeARA();
    ~SearchTreeARA();

    void addToOpen(SearchNode* node);
    void addToClosed(SearchNode* node);
    void addToIncons(SearchNode* node);

    void clearOpen();
    // actually do not delete nodes, only pushes to _history
    void clearClosed();
    void clearIncons();
    void clearHistory();

    bool wasExpanded(SearchNode* node) const;

    void updateOpen(CostType w);

    // returns best node and remove it from open
    SearchNode* extractBestNode();

    size_t size() const;
    size_t sizeOpen() const;

private:
    // sort by priority
    multiset<SearchNode*, CmpByPriority> _open;
    // sort by state
    set<SearchNode*, CmpByState> _closed;
    // sort by state
    set<SearchNode*, CmpByState> _incons;
    vector<SearchNode*> _history;
};

Solution lazyARAstar(
    const JointState& startPos,
    IAstarChecker& checker,
    double weight,
    double timeLimit
);

Solution lazyARAstar(
    const JointState& startPos,
    IAstarChecker& checker,
    Solution& startSolution,
    double weight,
    double timeLimit
);

} // namespace astar
