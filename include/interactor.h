#pragma once

#include "joint_state.h"
#include "planner.h"
#include "logger.h"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <stdio.h>

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

    const std::pair<JointState, JointState>& getTest(size_t i) const;
    const std::pair<JointState, JointState>& getNextTest();
    bool haveNextTest() const;

    size_t size() const;

private:
    std::vector<std::pair<JointState, JointState>> _tests;
    size_t _nextTestId;
    size_t _dof;
};

class Interactor
{
public:
    Interactor(const std::string& modelFilename);
    ~Interactor();

    void setUp();

    void step();

    void stepLoop(double duration);

    void show();

    void doMainLoop();

private:
    mjData* _data;
    mjModel* _model;

    mjvCamera _cam;
    mjvOption _opt;
    mjvScene _scn;
    mjrContext _con;
    GLFWwindow* _window;

    ManipulatorPlanner* _planner;
    Logger* _logger;
    TestSet* _testset;

    size_t _dof;
};

