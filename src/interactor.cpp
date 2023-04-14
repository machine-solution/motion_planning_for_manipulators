#include "interactor.h"
#include "global_defs.h"

#include <stdexcept>

TestSet::TestSet(size_t dof)
{
    _dof = dof;
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
        _tests.push_back({start, goal});
    }
    fclose(file);
}
void TestSet::generateRandomTests(size_t n, size_t seed)
{
    srand(seed);
    for (size_t i = 0; i < n; ++i)
    {
        _tests.push_back({randomState(_dof, g_units), randomState(_dof, g_units)});
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

size_t TestSet::size() const
{
    return _tests.size();
}

const std::pair<JointState, JointState>& TestSet::getTest(size_t i) const
{
    return _tests[i];
}
const std::pair<JointState, JointState>& TestSet::getNextTest()
{
    return _tests[_nextTestId++];
}
bool TestSet::haveNextTest() const
{
    return _nextTestId < _tests.size();
}


Interactor::Interactor(const std::string& modelFilename)
{
    char error[1000] = "Could not load binary model";
    _model = mj_loadXML(modelFilename.c_str(), 0, error, 1000);
    if (!_model)
        mju_error_s("Load model error: %s", error); // exception in constructor - bad idea TODO
    _data = mj_makeData(_model);
    _dof = _model->nq / 2;

    mjModel* mCopy = mj_copyModel(NULL, _model);
    mjData* dCopy = mj_makeData(mCopy);

    _planner = new ManipulatorPlanner(_dof, mCopy, dCopy);
    _logger = new Logger(_dof);
    _testset = new TestSet(_dof);
}
Interactor::~Interactor()
{
    // free MuJoCo model and data, deactivate
    mjv_freeScene(&_scn);
    mjr_freeContext(&_con);
    mj_deleteData(_data);
    mj_deleteModel(_model);
    delete _planner;
    delete _logger;
    delete _testset;
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void Interactor::setUp()
{
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

    _logger->prepareMainFile("");
    // _logger->prepareRuntimeFile("pyplot/4/runtime.log");
    _logger->prepareScenFile("scenaries/scen.log");
    _logger->prepareStatsFile("pyplot/4/stats.log");

    // _testset->generateRandomTests(1000);
    _testset->loadTests("scenaries/test.scen");

    printf("Simulation is started!\n");
}

void Interactor::step()
{
    static int counter = 0;
    static int partOfMove = 0;
    static bool haveToPlan = false;
    static int solved = 0;

    static Solution solution;
    static JointState currentState(_dof, 0);
    static JointState goal(_dof, 0);
    static JointState delta(_dof, 0);

    if (solution.goalAchieved())
    {
        if (!_testset->haveNextTest())
        {
            _shouldClose = true;
            return;
        }
        delta = JointState(_dof, 0);
        if (!haveToPlan)
        {
            // generating new test
            std::pair<JointState, JointState> test = _testset->getNextTest();
            currentState = test.first;
            goal = test.second;
            for (size_t i = 0; i < _dof; ++i)
            {
                _data->qpos[i + _dof] = goal.rad(i);
                _data->qpos[i] = currentState.rad(i);
            }
            haveToPlan = true;
        }
        else if (haveToPlan)
        {
            // planning path to goal
            ++counter;
            if (counter > 8) // to first of all simulator can show picture
            {
                counter = 0;
                solution = _planner->planSteps(currentState, goal, ALG_ASTAR, 600.0);
                haveToPlan = false;

                _logger->printMainLog(solution);
                
                _logger->printStatsLog(solution);
                _logger->printScenLog(solution, currentState, goal);
                
                printf("solved %d/%zu\n", ++solved, _testset->size());
            }
        }
    }
    else
    {
        if (partOfMove == g_unitSize - 1)
        {
            currentState += delta;
            for (size_t i = 0; i < _dof; ++i)
            {
                _data->qpos[i] = currentState.rad(i);
            }
            delta = solution.nextStep();
            partOfMove = 0;
        }
        else
        {
            ++partOfMove;
            for (size_t i = 0; i < _dof; ++i)
            {
                _data->qpos[i] += delta[i] * g_worldEps;
            }
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
