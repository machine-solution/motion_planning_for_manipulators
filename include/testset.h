#pragma once

#include "joint_state.h"

#include <string>

class Test
{
public:
    Test(const JointState& startPos, const JointState& goalPos);

    const JointState& start() const;
    const JointState& goal() const;
private:
    JointState _start;
    JointState _goal;
};

class TestSet
{
public:
    TestSet(size_t dof);
    TestSet(size_t dof, const std::string& filename);
    TestSet(size_t dof, size_t n, size_t seed = 12345);

    void loadTests(const std::string& filename);
    void generateRandomTests(size_t n, size_t seed = 12345);
    void removeTests();
    void restartTests();

    const Test& getTest(size_t i) const;
    const Test& getNextTest();
    bool haveNextTest() const;

    size_t progress() const;
    size_t size() const;

private:
    std::vector<Test> _tests;
    size_t _nextTestId;
    size_t _dof;
};
