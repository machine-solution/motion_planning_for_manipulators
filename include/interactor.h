#pragma once

#include "planner.h"
#include "logger.h"
#include "taskset.h"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <stdio.h>

struct Config
{
    std::string modelFilename;
    size_t dof;
    size_t arms;
    size_t totalGeoms;
    std::vector<size_t> collideGeomList;
    double timeLimit;
    double w;
    size_t constraintInterval;
    int taskNum;
    TaskType taskType;
    bool randomTasks;
    std::string scenFilename;
    std::string statsFilename;
    std::string tasksFilename;
    std::string runtimeFilename;
    std::string cSpaceFilename;
    std::string pathsFolder;
    bool displayMotion;
    Algorithm algorithm;
    Preprocess preprocess;
    int clusters;
    size_t randomSeed;
};

struct ModelState
{
    int counter = 0;
    int partOfMove = 0;
    int freezeCounter = 0;
    bool haveToPlan = false;
    bool needSetTask = true;
    MultiSolution solution;

    MultiState start;
    MultiState currentState;
    MultiState goal;
    MultiAction action;
    const ITask* task;
};

class Interactor
{
public:
    Interactor();
    ~Interactor();

    void logQPos(const std::string& text);

    void setUp(Config config);
    void setUp(const string& filename);

    void setManipulatorState(const MultiState& state);
    void setGoalState(const MultiState& state);
    // simulate action in currentState
    // at end stage aplies action to currentState
    // return next stage
    size_t simulateAction(MultiState& currentState, const MultiAction& action, size_t stage);

    void setTask();
    void solveTask();

    void cleanAcc();

    void step();

    void stepLoop(double duration);

    void show();

    bool shouldClose();

    void doMainLoop();

    void constructorStep();
    void doConstructorLoop();

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
    TaskSet* _taskset;

    size_t _dof;
    size_t _arms;

    bool _shouldClose = false;

    Config _config;
    ModelState _modelState;

    Config parseJSON(const string& filename);
};

