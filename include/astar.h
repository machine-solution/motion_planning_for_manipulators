#pragma once

#include "joint_state.h"
#include "utils.h"
#include "solution.h"

#include <set>

using std::set;
using std::multiset;

namespace astar
{

class SearchNode
{
public:
    SearchNode(
        CostType g,
        CostType h,
        CostType w,
        const JointState& state,
        int stepNum = -1,
        SearchNode* parent = nullptr,
        bool isLazy = false
    );

    CostType g() const;
    CostType h() const;
    CostType f() const;
    int stepNum() const;
    const JointState& state() const;
    SearchNode* parent();

    // set _isLazy
    void updateLazy(bool newLazy);
    bool isLazy() const;

    void updateWeight(CostType w);

    size_t byteSize() const;

    // sort by priority
    bool operator<(const SearchNode& sn);

protected:
    CostType _g, _h, _w;
    // number of step, which change parent.state() -> this.state(). -1 if have no parent
    int _stepNum;
    JointState _state;
    SearchNode* _parent;

    // True if node has unfinished checks
    bool _isLazy;
};

class CmpByState
{
public:
    bool operator()(SearchNode* a, SearchNode* b) const;
};
class CmpByPriority
{
public:
    bool operator()(SearchNode* a, SearchNode* b) const;
};

/*
This is a container and data structure for A* algorithm.
A* relies on this Tree in deletation nodes.
*/
class SearchTree : public Profiler
{
public:
    SearchTree();
    ~SearchTree();

    void addToOpen(SearchNode* node);
    void addToClosed(SearchNode* node);

    // returns best node and remove it from open
    SearchNode* extractBestNode();

    size_t size() const;
    size_t sizeOpen() const;

private:
    bool wasExpanded(SearchNode* node) const;
    // sort by priority
    multiset<SearchNode*, CmpByPriority> _open;
    // sort by state
    set<SearchNode*, CmpByState> _closed;
};

class IAstarChecker
{
public:
    virtual bool isCorrect(const JointState& state, const Action& action) = 0;
    virtual bool isGoal(const JointState& state) = 0;
    virtual CostType costAction(const JointState& state, const Action& action) = 0;
    virtual const std::vector<Action>& getActions() = 0;
    virtual const Action& getZeroAction() = 0;
    virtual CostType heuristic(const JointState& state) = 0;
};

// allocates on heap and returns successors
vector<SearchNode*> generateSuccessors(
    SearchNode* node,
    IAstarChecker& checker,
    double weight
);

Solution astar(
    const JointState& startPos,
    IAstarChecker& checker,
    double weight = 1.0,
    double timeLimit = 1.0
);

} // namespace astar
