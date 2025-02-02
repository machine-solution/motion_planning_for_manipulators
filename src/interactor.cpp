#include "global_defs.h"
#include "interactor.h"
#include "utils.h"

#include <external/json.h>

#include <iostream>
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
    _dof = _config.dof;
    _arms = config.arms;

    mjModel* mCopy = mj_copyModel(NULL, _model);
    mjData* dCopy = mj_makeData(mCopy);

    _planner = new ManipulatorPlanner(_dof, _arms, mCopy, dCopy);
    // _planner->preprocess(); TODO return
    _logger = new Logger(_dof);
    _taskset = new TaskSet(_dof, _arms);

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

    _modelState.start = MultiState(_dof, _arms, 0);
    _modelState.currentState = MultiState(_dof, _arms, 0);
    _modelState.goal = MultiState(_dof, _arms, 0);
    _modelState.action = MultiAction(_dof, _arms, 0);

    _logger->prepareMainFile("");
    _logger->prepareScenFile(_config.scenFilename);
    _logger->prepareStatsFile(_config.statsFilename);
    _logger->prepareRuntimeFile(_config.runtimeFilename);
    if (_dof == 2 && _arms == 1)
    {
        // _logger->prepareCspaceFile(_config.cSpaceFilename);
        _logger->printCSpace(_planner->configurationSpace());
        // _logger->preparePathsFolder(_config.pathsFolder);
    }

    _planner->preprocess(_config.preprocess, _config.clusters, _config.randomSeed);

    if (_config.randomTasks)
    {
        _taskset->generateRandomTasks(_config.taskNum, _config.taskType, *_planner, _config.randomSeed);
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

void Interactor::setManipulatorState(const MultiState& state)
{
    for (size_t a = 0; a < _arms; ++a)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i + a * _dof] = state[a].rad(i);
        }
    }
}
void Interactor::setGoalState(const MultiState& state)
{
    for (size_t a = 0; a < _arms; ++a)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i + a * _dof + _arms * _dof] = state[a].rad(i);
        }
    }
}
size_t Interactor::simulateAction(MultiState& currentState, const MultiAction& action, size_t stage)
{
    if (stage == g_unitSize - 1)
    {
        currentState.apply(action);
        setManipulatorState(currentState);
        return 0;
    }
    else
    {
        for (size_t a = 0; a < _arms; ++a)
        {
            for (size_t i = 0; i < _dof; ++i)
            {
                _data->qpos[i + a * _dof] += action[a][i] * g_worldEps;
            }
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
        _modelState.start = static_cast<const TaskState*>(_modelState.task)->start();
        _modelState.currentState = _modelState.start;
        _modelState.goal = static_cast<const TaskState*>(_modelState.task)->goal();
        // if correct task TODO remove
        if (!_planner->checkMultiCollision(_modelState.currentState) && !_planner->checkMultiCollision(_modelState.goal))
        {
            setManipulatorState(_modelState.currentState);
            setGoalState(_modelState.goal);
            _modelState.haveToPlan = true;
        }
    }
    else if (_modelState.task->type() == TASK_POSITION)
    {
        _modelState.start = static_cast<const TaskPosition*>(_modelState.task)->start();
        _modelState.currentState = _modelState.start;
        // if correct task TODO remove
        if (!_planner->checkMultiCollision(_modelState.currentState))
        {
            setManipulatorState(_modelState.currentState);
            _modelState.haveToPlan = true;
        }
    }
}
void Interactor::solveTask()
{
    // planning path to goal
    if (_modelState.task->type() == TASK_STATE)
    {
        _modelState.solution = _planner->planMultiActions(_modelState.currentState, _modelState.goal,
            _config.algorithm, _config.timeLimit, _config.w);

        // _logger->printScenLog(_modelState.solution, _modelState.currentState, _modelState.goal);
    }
    else if (_modelState.task->type() == TASK_POSITION)
    {
        // _modelState.solution = _planner->planActions(_modelState.currentState,
        //     static_cast<const TaskPosition*>(_modelState.task)->goalX(),
        //     static_cast<const TaskPosition*>(_modelState.task)->goalY(),
        //     _config.algorithm, _config.timeLimit, _config.w);

        // _logger->printScenLog(_modelState.solution, _modelState.currentState, 
        //     static_cast<const TaskPosition*>(_modelState.task)->goalX(),
        //     static_cast<const TaskPosition*>(_modelState.task)->goalY());
    }
    _modelState.haveToPlan = false;

    // _logger->printMainLog(_modelState.solution);
    
    // _logger->printStatsLog(_modelState.solution);

    // _logger->printRuntimeLog(_modelState.solution);

    if (_dof == 2)
    {
        // _logger->printPath(
        //     _planner->pathInConfigurationSpace(
        //         _modelState.start,
        //         _modelState.solution
        //     )
        // );
    }
    
    printf("progress %zu/%zu\n\n", _taskset->progress(), _taskset->size());
    
}

void Interactor::step()
{
    if (_shouldClose)
    {
        return;
    }
    if (_modelState.needSetTask)
    {
        _modelState.action = MultiAction(_dof, _arms, 0);
        if (_config.displayMotion && _modelState.freezeCounter++ < 256)
        {
            
        }
        else
        {
            setTask();
            _modelState.needSetTask = false;
            _modelState.haveToPlan = true;
            _modelState.freezeCounter = 0;
        }
    }
    else if (_modelState.haveToPlan)
    {
        _modelState.action = MultiAction(_dof, _arms, 0);

        if (_config.displayMotion && _modelState.freezeCounter++ < 256)
        {
            
        }
        else
        {
            solveTask();
            _modelState.haveToPlan = false;
            _modelState.partOfMove = 0;
            _modelState.freezeCounter = 0;
        }
    }
    else
    {
        if (!_config.displayMotion)
        {
            _modelState.needSetTask = true;
        }
        else
        {
            if (_modelState.partOfMove == 0)
            {
                if (_modelState.solution.goalAchieved())
                {
                    _modelState.needSetTask = true;
                }
                else
                {
                    _modelState.action = _modelState.solution.nextAction();
                }
            }
            _modelState.partOfMove = simulateAction(_modelState.currentState, _modelState.action, _modelState.partOfMove);
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

    std::string modelFilename = data["model"]["filename"];
    size_t dof = data["model"]["dof"];
    size_t arms = data["model"]["arms"];
    double timeLimit = data["algorithm"]["time_limit"];
    Algorithm algorithm = data["algorithm"]["type"];
    double w = data["algorithm"]["heuristic"]["weight"];
    int taskNum = data["taskset"]["task_number"];
    TaskType taskType = data["taskset"]["task_type"];
    bool randomTasks = data["taskset"]["use_random_tasks"];
    size_t randomSeed = data["taskset"]["random_seed"];
    std::string scenFilename = data["output"]["taskset"];
    std::string statsFilename = data["output"]["statistics"];
    std::string tasksFilename = data["taskset"]["taskset_filename"];
    std::string runtimeFilename = data["output"]["profiling"];
    std::string cSpaceFilename = data["output"]["configuration_space"];
    std::string pathsFolder = data["output"]["paths_folder"];
    Preprocess preprocess = data["preprocess"]["type"];
    int clusters = data["preprocess"]["clusters"];
    bool displayMotion = data["display_motion"];

    return Config{
        modelFilename,
        dof,
        arms,
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
        pathsFolder,
        displayMotion,
        algorithm,
        preprocess,
        clusters,
        randomSeed,
    };
}
