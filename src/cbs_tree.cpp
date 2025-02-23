#include "cbs_tree.h"

#include <cmath>
#include <random>
#include <iostream>

size_t CBSNode::_nextId = 0;

CBSNode::CBSNode(
    size_t conflictCount, CostType sumG, const std::vector<ConstraintList> &parentConstraintsMap,
    const MultiSolution &parentSolution, std::shared_ptr<IConstraint> newConstraint, size_t armNum, CBSNode *parent, bool isLazy)
{
    _id = (_nextId++);
    _conflictCount = conflictCount;
    _sumG = sumG;
    _isLazy = isLazy;
    _constraintsMap = parentConstraintsMap;
    if (newConstraint->type() < CONSTRAINT_MAX)
    {
        _constraintsMap[armNum].push_back(newConstraint);
    }
    _armNum = armNum;
    _constraintAddedType = newConstraint->type();
    _parent = parent;
    _solution = parentSolution;

    _newConstraintType = newConstraint->type();
}

ConstraintType CBSNode::newConstraintType() const
{
    return _newConstraintType;
}

size_t CBSNode::conflictCount() const
{
    return _conflictCount;
}

CostType CBSNode::sumG() const
{
    return _sumG;
}

MultiSolution CBSNode::solution() const
{
    return _solution;
}

ConstraintList CBSNode::constraints(size_t armNum) const
{
    return _constraintsMap.at(armNum);
}

std::vector<ConstraintList> CBSNode::constraintsMap() const
{
    return _constraintsMap;
}

size_t CBSNode::armNum() const
{
    return _armNum;
}

CBSNode *CBSNode::parent()
{
    return _parent;
}

void CBSNode::updateLazy(size_t conflictCount, CostType sumG, const MultiSolution &solution)
{
    _isLazy = false;
    _solution = solution;
    _sumG = sumG,
    _conflictCount = conflictCount;
}

bool CBSNode::isLazy() const
{
    return _isLazy;
}

std::vector<double> CBSNode::priorityConflicts(ConstraintType constraintType) const
{
    size_t typedConstraints = 0;
    size_t allConstraints = 0;
    for (auto constraints : _constraintsMap)
    {
        for (auto cst : constraints)
        {
            if (cst->type() == constraintType)
            {
                ++typedConstraints;
            }
            ++allConstraints;
        }
    }
    double cstRatio = 0;
    if (allConstraints > 0)
    {
        cstRatio = (double)typedConstraints / allConstraints;
    }
    return std::vector<double>({(double)conflictCount(), sumG(), 1 - cstRatio, (double)id()});
}

std::pair<double, size_t> CBSNode::prioritySumG() const
{
    return std::pair<double, size_t>({sumG(), id()});
}

size_t CBSNode::id() const
{
    return _id;
}

bool CmpByStateCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->id() < b->id();
}

bool CmpBySumGCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->prioritySumG() < b->prioritySumG();
}

bool CmpVertexConflictsCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->priorityConflicts(CONSTRAINT_VERTEX) < b->priorityConflicts(CONSTRAINT_VERTEX);
}

bool CmpAvoidanceConflictsCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->priorityConflicts(CONSTRAINT_AVOIDANCE) < b->priorityConflicts(CONSTRAINT_AVOIDANCE);
}

bool CmpSphereConflictsCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->priorityConflicts(CONSTRAINT_SPHERE) < b->priorityConflicts(CONSTRAINT_SPHERE);
}

bool CmpPriorityConflictsCBS::operator()(CBSNode *a, CBSNode *b) const
{
    return a->priorityConflicts(CONSTRAINT_PRIORITY) < b->priorityConflicts(CONSTRAINT_PRIORITY);
}


CBSTree::CBSTree(double w) : _focal(w)
{
    _w = w;
    for (int type = 0; type < (int)CONSTRAINT_MAX; ++type)
    {
        _rewardStorages.push_back(RewardStorage(g_maxC));
    }
}
CBSTree::~CBSTree()
{
    while (!_open.empty())
    {
        CBSNode* node = *_open.begin();
        _open.erase(_open.begin());
        delete node;
    }

    while (!_closed.empty())
    {
        CBSNode* node = *_closed.begin();
        _closed.erase(_closed.begin());
        delete node;
    }
}

void CBSTree::addToOpen(CBSNode* node)
{
    startProfiling();
    if (node->sumG() <= _w * _focal.minSumG())
    {
        _focal.add(node);
    }
    else
    {
        _open.insert(node);
    }
    stopProfiling();
}
void CBSTree::addToClosed(CBSNode* node)
{
    startProfiling();
    _closed.insert(node);
    stopProfiling();
}

CBSNode* CBSTree::extractBestNode()
{
    startProfiling();
    CBSNode* returnNode = _focal.extractBestNode(sampleK());
    while (!_open.empty() && (*_open.begin())->sumG() <= _w * _focal.minSumG())
    {
        CBSNode* openNode = *_open.begin();
        _open.erase(openNode);
        _focal.add(openNode);
    }
    stopProfiling();
    return returnNode;
}

// Генерация случайного числа от 0 до 1
double uniformRandom() {
    return rand() / (double)RAND_MAX;
}

// Генерация sэмплов из гамма-распределения (Gamma(alpha, 1))
double gammaSample(double alpha) {
    if (alpha < 1.0) {
        double u = uniformRandom();
        return gammaSample(1.0 + alpha) * std::pow(u, 1.0 / alpha);
    }

    double d = alpha - 1.0 / 3.0;
    double c = 1.0 / std::sqrt(9.0 * d);

    while (true) {
        double x, v;
        do {
            x = std::sqrt(-2.0 * std::log(uniformRandom())) * cos(2.0 * M_PI * uniformRandom());
            v = (1.0 + c * x) * (1.0 + c * x) * (1.0 + c * x);
        } while (v <= 0);

        double u = uniformRandom();
        if (u < 1.0 - 0.0331 * (x * x) * (x * x)) 
            return d * v;
        
        if (std::log(u) < 0.5 * x * x + d * (1.0 - v + std::log(v)))
            return d * v;
    }
}

// Сэмплирование из бета-распределения Beta(alpha, beta)
double betaSample(double alpha, double beta) {
    double x = gammaSample(alpha);
    double y = gammaSample(beta);
    return x / (x + y);
}

ConstraintType CBSTree::sampleK()
{
    std::vector<double> values((size_t)CONSTRAINT_MAX);
    size_t argMax = 0;
    for (size_t i = 0; i < (size_t)CONSTRAINT_MAX; ++i)
    {
        values[i] = betaSample(_rewardStorages[i].rewards(), _rewardStorages[i].penalty());
        if (values[i] > values[argMax])
        {
            argMax = i;
        }
    }
    return (ConstraintType)argMax;
}

void CBSTree::payReward(ConstraintType k)
{
    _rewardStorages[k].payReward();
}

void CBSTree::payPenalty(ConstraintType k)
{
    _rewardStorages[k].payPenalty();
}

size_t CBSTree::size() const
{
    return _open.size() + _closed.size();
}
size_t CBSTree::sizeOpen() const
{
    return _open.size();
}

size_t CBSTree::sizeFocal() const
{
    return _focal.size();
}

bool CBSTree::wasExpanded(CBSNode* node) const
{
    startProfiling();
    bool res = _closed.count(node);
    stopProfiling();
    return res;
}

Focal::Focal(double w)
{
    _w = w;
    _minSumG = g_inf;
}

void Focal::add(CBSNode *node)
{
    _setSumG.insert(node);
    _setVertexConflicts.insert(node);
    _setAvoidanceConflicts.insert(node);
    _setSphereConflicts.insert(node);
    _setPriorityConflicts.insert(node);
    recalcMinSumG();
}

CBSNode *Focal::extractBestNode(ConstraintType k)
{
    if (_setSumG.empty())
    {
        return nullptr;
    }
    CBSNode* node = nullptr;
    switch (k)
    {
        case CONSTRAINT_VERTEX:
        node = *_setVertexConflicts.begin();
        break;
        case CONSTRAINT_AVOIDANCE:
        node = *_setAvoidanceConflicts.begin();
        break;
        case CONSTRAINT_SPHERE:
        node = *_setSphereConflicts.begin();
        break;
        case CONSTRAINT_PRIORITY:
        node = *_setPriorityConflicts.begin();
        break;
    }
    _setSumG.erase(node);
    _setVertexConflicts.erase(node);
    _setAvoidanceConflicts.erase(node);
    _setSphereConflicts.erase(node);
    _setPriorityConflicts.erase(node);
    recalcMinSumG();
    return node;
}

CostType Focal::minSumG() const
{
    return _minSumG;
}

size_t Focal::size() const
{
    return _setSumG.size();
}

void Focal::recalcMinSumG()
{
    if (_setSumG.empty())
    {
        _minSumG = g_inf;
    }
    else
    {
        _minSumG = (*_setSumG.begin())->sumG();
    }
}

RewardStorage::RewardStorage(size_t maxC)
{
    _maxC = maxC;
    _penalty = 0;
    _rewards = 0;
}

void RewardStorage::payReward()
{
    if (_memory.size() == _maxC)
    {
        int out = _memory.back();
        _memory.pop_back();
        if (out == 0)
        {
            --_penalty;
        }
        else
        {
            --_rewards;
        }
    }
    _memory.push_front(1);
    ++_rewards;
}

void RewardStorage::payPenalty()
{
    if (_memory.size() == _maxC)
    {
        int out = _memory.back();
        _memory.pop_back();
        if (out == 0)
        {
            --_penalty;
        }
        else
        {
            --_rewards;
        }
    }
    _memory.push_front(1);
    ++_penalty;
}

size_t RewardStorage::rewards() const
{
    return _rewards;
}

size_t RewardStorage::penalty() const
{
    return _penalty;
}
