#pragma once

#include "global_defs.h"
#include "solution.h"


class IPreprocChecker
{
public:
    virtual bool isCorrect(const JointState& state, const Action& action) = 0;
    virtual bool isCorrect(const JointState& state) = 0;
    virtual CostType costAction(const JointState& state, const Action& action) = 0;
    virtual const std::vector<Action>& getActions() = 0;
    virtual const Action& getZeroAction() = 0;
};


class IPreprocessor : public Profiler
{
public:
    IPreprocessor();
    ~IPreprocessor();

    virtual size_t memorySize() const = 0;

    virtual void preprocess(IPreprocChecker& checker) = 0;
    virtual Solution planActions(const JointState& startPos, const JointState goalPos) = 0;
};


