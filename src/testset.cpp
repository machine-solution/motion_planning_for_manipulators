#include "testset.h"

#include <cstdio>
#include <stdexcept>

Test::Test(const JointState& startPos, const JointState& goalPos)
{
    _start = startPos;
    _goal = goalPos;
}

const JointState& Test::start() const
{
    return _start;
}
const JointState& Test::goal() const
{
    return _goal;
}

TestSet::TestSet(size_t dof)
{
    _dof = dof;
    _nextTestId = 0;
}
TestSet::TestSet(size_t dof, const std::string& filename) : TestSet(dof)
{
    loadTests(filename);
}
TestSet::TestSet(size_t dof, size_t n, size_t seed) : TestSet(dof)
{
    generateRandomTests(n, seed);
}

void TestSet::loadTests(const std::string& filename)
{
    FILE* file = fopen(filename.c_str(), "r");
    // This may be called in constructor and
    // exceptions in constructor is a bad idea
    if (file == nullptr)
    {
        throw std::runtime_error("TestSet::loadTests: Could not open file " + filename);
    }
    int dof;
    fscanf(file, "%d", &dof);
    if (dof != _dof)
    {
        throw std::runtime_error("TestSet::loadTests: dof in testfile and in class are not same");
    }
    while (!feof(file))
    {
        JointState start(dof);
        JointState goal(dof);
        for (size_t i = 0; i < dof; ++i)
        {
            fscanf(file, "%d", &start[i]);
        }
        for (size_t i = 0; i < dof; ++i)
        {
            fscanf(file, "%d", &goal[i]);
        }
        float optimal;
        fscanf(file, "%f", &optimal); // it is really unused now
        _tests.push_back(Test(start, goal));
    }
    fclose(file);
}
void TestSet::generateRandomTests(size_t n, size_t seed)
{
    srand(seed);
    for (size_t i = 0; i < n; ++i)
    {
        _tests.push_back(Test(randomState(_dof, g_units), randomState(_dof, g_units)));
    }
}
void TestSet::removeTests()
{
    _tests.clear();
    _nextTestId = 0;
}
void TestSet::restartTests()
{
    _nextTestId = 0;
}

const Test& TestSet::getTest(size_t i) const
{
    return _tests[i];
}
const Test& TestSet::getNextTest()
{
    return _tests[_nextTestId++];
}
bool TestSet::haveNextTest() const
{
    return _nextTestId < _tests.size();
}

size_t TestSet::progress() const
{
    return _nextTestId;
}
size_t TestSet::size() const
{
    return _tests.size();
}

