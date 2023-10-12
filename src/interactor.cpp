#include "global_defs.h"
#include "interactor.h"
#include "utils.h"

#include <external/json.h>

#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

Interactor::Interactor() {}
Interactor::~Interactor()
{
    // free MuJoCo model and data, deactivate
    mjv_freeScene(&_scn);
    mjr_freeContext(&_con);
    mj_deleteData(_data);
    mj_deleteModel(_model);
    delete _planner;
    delete _logger;
    delete _taskset;
    mj_deactivate();

    // close glfw window
    glfwDestroyWindow(_window);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void Interactor::setUp(Config config)
{
    _config = config;

    char error[1000] = "Could not load binary model";
    _model = mj_loadXML(_config.modelFilename.c_str(), 0, error, 1000);
    if (!_model)
        mju_error_s("Load model error: %s", error); // exception in constructor - bad idea TODO
    _data = mj_makeData(_model);
    _dof = _model->nq / 2;

    mjModel* mCopy = mj_copyModel(NULL, _model);
    mjData* dCopy = mj_makeData(mCopy);

    _planner = new ManipulatorPlanner(_dof, mCopy, dCopy);
    _logger = new Logger(_dof);
    _taskset = new TaskSet(_dof);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    _window = glfwCreateWindow(1244, 700, "Manipulator", NULL, NULL);
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&_cam);
    mjv_defaultOption(&_opt);
    mjv_defaultScene(&_scn);
    mjr_defaultContext(&_con);
    mjv_makeScene(_model, &_scn, 2000);                // space for 2000 objects
    mjr_makeContext(_model, &_con, mjFONTSCALE_150);   // model-specific context

    // init camera
    double arr_view[] = {90, -90, 5.5, 0.000000, 0.000000, 0.000000};
    _cam.azimuth = arr_view[0];
    _cam.elevation = arr_view[1];
    _cam.distance = arr_view[2];
    _cam.lookat[0] = arr_view[3];
    _cam.lookat[1] = arr_view[4];
    _cam.lookat[2] = arr_view[5];

    _modelState.currentState = JointState(_dof, 0);
    _modelState.goal = JointState(_dof, 0);
    _modelState.action = Action(_dof, 0);

    _logger->prepareMainFile("");
    _logger->prepareScenFile(_config.scenFilename);
    _logger->prepareStatsFile(_config.statsFilename);
    _logger->prepareRuntimeFile(_config.runtimeFilename);
    if (_dof == 2)
    {
        _logger->prepareCspaceFile(_config.cSpaceFilename);
        _logger->printCSpace(_planner->configurationSpace());
    }

    if (_config.randomTasks)
    {
        _taskset->generateRandomTasks(_config.taskNum, _config.taskType);
    }
    else
    {
        _taskset->loadTasks(_config.tasksFilename, _config.taskType);
    }

    printf("Task count = %zu.\n", _taskset->size());
    printf("Simulation is started!\n\n");
}
void Interactor::setUp(const string& filename)
{
    setUp(parseJSON(filename));
}

void Interactor::setManipulatorState(const JointState& state)
{
    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = state.rad(i);
    }
}
void Interactor::setGoalState(const JointState& state)
{
    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i + _dof] = state.rad(i);
    }
}
size_t Interactor::simulateAction(JointState& currentState, const Action& action, size_t stage)
{
    if (stage == g_unitSize - 1)
    {
        currentState.apply(action);
        setManipulatorState(currentState);
        return 0;
    }
    else
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i] += action[i] * g_worldEps;
        }
        return stage + 1;
    }
}

void Interactor::setTask()
{
    // generating new task
    if (!_taskset->haveNextTask())
    {
        _shouldClose = true;
        return;
    }
    _modelState.task = _taskset->getNextTask();
    if (_modelState.task->type() == TASK_STATE)
    {
        _modelState.currentState = static_cast<const TaskState*>(_modelState.task)->start();
        _modelState.goal = static_cast<const TaskState*>(_modelState.task)->goal();
        // if correct task TODO remove
        if (!_planner->checkCollision(_modelState.currentState) && !_planner->checkCollision(_modelState.goal))
        {
            setManipulatorState(_modelState.currentState);
            setGoalState(_modelState.goal);
            _modelState.haveToPlan = true;
        }
    }
    else if (_modelState.task->type() == TASK_POSITION)
    {
        _modelState.currentState = static_cast<const TaskPosition*>(_modelState.task)->start();
        // if correct task TODO remove
        if (!_planner->checkCollision(_modelState.currentState))
        {
            setManipulatorState(_modelState.currentState);
            _modelState.haveToPlan = true;
        }
    }
}
void Interactor::solveTask()
{
    // planning path to goal
    ++_modelState.counter;
    if (_modelState.counter > 8) // to first of all simulator can show picture
    {
        _modelState.counter = 0;
        if (_modelState.task->type() == TASK_STATE)
        {
            _modelState.solution = _planner->planActions(_modelState.currentState, _modelState.goal,
                _config.algorithm, _config.timeLimit, _config.w);

            _logger->printScenLog(_modelState.solution, _modelState.currentState, _modelState.goal);
        }
        else if (_modelState.task->type() == TASK_POSITION)
        {
            _modelState.solution = _planner->planActions(_modelState.currentState,
                static_cast<const TaskPosition*>(_modelState.task)->goalX(),
                static_cast<const TaskPosition*>(_modelState.task)->goalY(),
                _config.algorithm, _config.timeLimit, _config.w);

            _logger->printScenLog(_modelState.solution, _modelState.currentState, 
                static_cast<const TaskPosition*>(_modelState.task)->goalX(),
                static_cast<const TaskPosition*>(_modelState.task)->goalY());
        }
        _modelState.haveToPlan = false;

        _logger->printMainLog(_modelState.solution);
        
        _logger->printStatsLog(_modelState.solution);

        _logger->printRuntimeLog(_modelState.solution);
        
        printf("progress %zu/%zu\n\n", _taskset->progress(), _taskset->size());
    }
}

void Interactor::step()
{
    if (!_config.displayMotion || _modelState.solution.goalAchieved())
    {
        _modelState.action = Action(_dof, 0);
        if (!_modelState.haveToPlan)
        {
            setTask();
        }
        else if (_modelState.haveToPlan)
        {
            solveTask();
        }
    }
    else
    {
        _modelState.partOfMove = simulateAction(_modelState.currentState, _modelState.action, _modelState.partOfMove);
        if (_modelState.partOfMove == 0)
        {
            _modelState.action = _modelState.solution.nextAction();
        }
    }

    mj_step(_model, _data);
}

void Interactor::stepLoop(double duration)
{
    mjtNum simstart = _data->time;
    while (_data->time - simstart < duration && !shouldClose())
    {
        step();
        if (_data->ncon)
        {
            printf("Collision detected! Acc[0] = (%f)\n", fabs(_data->qacc[0]));
        }
    }
}

void Interactor::show()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(_window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(_model, _data, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
    mjr_render(viewport, &_scn, &_con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(_window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

bool Interactor::shouldClose()
{
    return glfwWindowShouldClose(_window) || _shouldClose;
}

void Interactor::doMainLoop()
{
    const double fps = 60.0;
    while (!shouldClose())
    {
        stepLoop(1 / fps);
        show();
    }
}


Config Interactor::parseJSON(const string& filename)
{
    std::ifstream fin(filename);
    json data = json::parse(fin);

    std::string modelFilename = data["model_filename"];
    double timeLimit = data["algorithm"]["time_limit"];
    double w = data["algorithm"]["weight"];
    Algorithm algorithm = data["algorithm"]["type"];
    int taskNum = data["taskset"]["task_number"];
    TaskType taskType = data["taskset"]["task_type"];
    bool randomTasks = data["taskset"]["use_random_tasks"];
    std::string scenFilename = data["output"]["taskset"];
    std::string statsFilename = data["output"]["statistics"];
    std::string tasksFilename = data["taskset"]["taskset_filename"];
    std::string runtimeFilename = data["output"]["profiling"];
    std::string cSpaceFilename = data["output"]["configuration_space"];
    bool displayMotion = data["display_motion"];

    return Config{
        modelFilename,
        timeLimit,
        w,
        taskNum,
        taskType,
        randomTasks,
        scenFilename,
        statsFilename,
        tasksFilename,
        runtimeFilename,
        cSpaceFilename,
        displayMotion,
        algorithm
    };
}
