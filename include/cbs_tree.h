#pragma once

#include "constraint.h"
#include "joint_state.h"
#include "utils.h"
#include "taskset.h"
#include "solution.h"
#include "global_defs.h"

#include <set>
#include <map>
#include <deque>

class CBSNode
{
public:
    CBSNode(
        size_t conflictCount,
        CostType sumG,
        const std::vector<ConstraintList>& parentConstraintsMap,
        const MultiSolution& parentSolution,
        std::shared_ptr<IConstraint> newConstraint,
        size_t armNum,
        CBSNode* parent = nullptr,
        bool isLazy = true
    );
    
    ConstraintType newConstraintType() const;

    size_t conflictCount() const;
    CostType sumG() const;
    MultiSolution solution() const;

    ConstraintList constraints(size_t armNum) const;
    std::vector<ConstraintList> constraintsMap() const;
    size_t armNum() const;
    CBSNode* parent();

    // set _isLazy
    void updateLazy(size_t conflictCount, CostType sumG, const MultiSolution& solution);
    bool isLazy() const;

    std::vector<double> priorityConflicts(ConstraintType constraintType) const;
    std::pair<double, size_t> prioritySumG() const;

    size_t id() const;

protected:
    size_t _conflictCount;
    CostType _sumG;

    MultiSolution _solution;
    std::vector<ConstraintList> _constraintsMap;
    // number of the arm to which was added new constraint from parent
    size_t _armNum;
    // Type of constraint was added from parent
    ConstraintType _constraintAddedType;

    CBSNode* _parent;

    // True if node has unfinished planning
    bool _isLazy;

    size_t _id;

    static size_t _nextId;

    ConstraintType _newConstraintType;
};

class CmpByStateCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};
class CmpBySumGCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};
class CmpVertexConflictsCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};
class CmpAvoidanceConflictsCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};
class CmpSphereConflictsCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};
class CmpPriorityConflictsCBS
{
public:
    bool operator()(CBSNode* a, CBSNode* b) const;
};


class Focal
{
public:
    Focal(double w);

    void add(CBSNode* node);

    // returns best node and remove it from himself
    CBSNode* extractBestNode(ConstraintType k);

    CostType minSumG() const;
    size_t size() const;

private:
    void recalcMinSumG();

    double _w;
    CostType _minSumG = 0;

    multiset<CBSNode*, CmpVertexConflictsCBS> _setVertexConflicts;
    multiset<CBSNode*, CmpAvoidanceConflictsCBS> _setAvoidanceConflicts;
    multiset<CBSNode*, CmpSphereConflictsCBS> _setSphereConflicts;
    multiset<CBSNode*, CmpPriorityConflictsCBS> _setPriorityConflicts;

    multiset<CBSNode*, CmpBySumGCBS> _setSumG;
};

class RewardStorage
{
public:
    RewardStorage(size_t maxC);

    void payReward();
    void payPenalty();

    size_t rewards() const;
    size_t penalty() const;

private:
    std::deque<int> _memory;
    size_t _maxC;
    size_t _rewards;
    size_t _penalty;
};

/*
This is a container and data structure for A* algorithm.
A* relies on this Tree in deletation nodes.
*/
class CBSTree : public Profiler
{
public:
    CBSTree(double w);
    ~CBSTree();

    void addToOpen(CBSNode* node);
    void addToClosed(CBSNode* node);

    // returns best node and remove it from open
    CBSNode* extractBestNode();

    ConstraintType sampleK();
    void payReward(ConstraintType k);
    void payPenalty(ConstraintType k);

    size_t size() const;
    size_t sizeOpen() const;
    size_t sizeFocal() const;

private:
    bool wasExpanded(CBSNode* node) const;
    // sort by sumG
    multiset<CBSNode*, CmpBySumGCBS> _open;
    // sort by state
    set<CBSNode*, CmpByStateCBS> _closed;
    // sort by conflicts
    Focal _focal;
    double _w;

    std::vector<RewardStorage> _rewardStorages;
};
