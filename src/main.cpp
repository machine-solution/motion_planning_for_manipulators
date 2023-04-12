#include "planner.h"

#include <stdbool.h> 
#include <set>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

char modelFilename[] = "model/4-dof/manipulator_4.xml";
char testsFilename[] = "scenaries/4_hard.scen";
char resFilename[] = "pyplot/stats.log";
char scenFilename[] = "scenaries/scen.log";

FILE* logFile = nullptr;
FILE* scenFile = nullptr;
FILE* testsFile = nullptr;

vector<std::pair<JointState, JointState>> g_tests;

const int seed = 12345;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void printLog(FILE* file, const Solution& solution)
{
    std::string yn[] = {"FOUND", "NOT FOUND", "NOT EXISTS"};

    fprintf(file, "path verdict: %s\nexpansions: %zu\nmax tree size: %zu\ncost of path: %f\nruntime: %.3fs\n",
        yn[solution.stats.pathVerdict].c_str(),
        solution.stats.expansions,
        solution.stats.maxTreeSize,
        solution.stats.pathCost,
        solution.stats.runtime
    );
    fprintf(file, "---Planner Profile---\n");
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%.1fms\t%zu\t%s\n",
            info.runtime * 1000,
            info.calls,
            info.funcName.c_str()
        );
    }
    fprintf(file, "---Search Tree Profile---\n");
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%.1fms\t%zu\t%s\n",
            info.runtime * 1000,
            info.calls,
            info.funcName.c_str()
        );
    }
    fprintf(file, "\n");
}

void printRuntimeLogHeader(FILE* file, const Solution& solution)
{
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%s_time,", info.funcName.c_str());
        fprintf(file, "%s_calls,", info.funcName.c_str());
    }
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%s_time,", info.funcName.c_str());
        fprintf(file, "%s_calls,", info.funcName.c_str());
    }
    fprintf(file, "whole_runtime\n");
}

void printRuntimeLog(FILE* file, const Solution& solution)
{
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%f,", info.runtime * 1000);
        fprintf(file, "%ld,", info.calls);
    }
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%f,", info.runtime * 1000);
        fprintf(file, "%ld,", info.calls);
    }
    fprintf(file, "%f\n", solution.stats.runtime * 1000);
}

void printStatsLogHeader(FILE* file, const Solution& solution)
{
    fprintf(file, "expansions,runtime,maxTreeSize,pathCost,pathFound\n");
}

void printStatsLog(FILE* file, const Solution& solution)
{
    fprintf(file, "%zu,%f,%zu,%f,%d\n",
        solution.stats.expansions,
        solution.stats.runtime,
        solution.stats.maxTreeSize,
        solution.stats.pathCost,
        solution.stats.pathVerdict);
}

void printSceneLogHeader(FILE* file, const Solution& solution, const JointState& startPos, const JointState& goalPos)
{
    for (size_t i = 0; i < startPos.dof(); ++i)
    {
        fprintf(file, "start_%zu,", i);
    }
    for (size_t i = 0; i < goalPos.dof(); ++i)
    {
        fprintf(file, "goal_%zu,", i);
    }
    fprintf(file, "path_cost,difficulty,runtime\n");
}

void printSceneLog(FILE* file, const Solution& solution, const JointState& startPos, const JointState& goalPos)
{
    for (size_t i = 0; i < startPos.dof(); ++i)
    {
        fprintf(file, "%d,", startPos[i]);
    }
    for (size_t i = 0; i < goalPos.dof(); ++i)
    {
        fprintf(file, "%d,", goalPos[i]);
    }
    fprintf(file, "%f,%f,%f\n", solution.stats.pathCost,
        1.0 * solution.stats.pathCost / solution.stats.pathPotentialCost,
        solution.stats.runtime);
}

void printCSpace(FILE* file, const vector<string>& cSpace)
{
    for (size_t i = 0; i < cSpace.size(); ++i)
    {
        fprintf(file, "%s\n", cSpace[i].c_str());
    }
}

vector<std::pair<JointState, JointState>> loadTests(FILE* file)
{
    int n;
    fscanf(file, "%d", &n);
    vector<std::pair<JointState, JointState>> tests;
    while (!feof(file))
    {
        JointState start(n);
        JointState goal(n);
        for (size_t i = 0; i < n; ++i)
        {
            fscanf(file, "%d", &start[i]);
        }
        for (size_t i = 0; i < n; ++i)
        {
            fscanf(file, "%d", &goal[i]);
        }
        float optimal;
        fscanf(file, "%f", &optimal); // it is really unused now
        tests.push_back({start, goal});
    }
    return tests;
}

void planner_step_tests(mjModel* m, mjData* d, ManipulatorPlanner& planner)
{
    static int counter = 0;
    static int partOfMove = 0;
    static bool haveToPlan = false;
    static int solved = 0;

    static Solution solution;
    static JointState currentState(planner.dof(), 0);
    static JointState goal(planner.dof(), 0);
    static JointState delta(planner.dof(), 0);

    if (solution.goalAchieved())
    {
        if (solved == g_tests.size() - 1)
        {
            fclose(logFile);
            fclose(scenFile);
            fclose(testsFile);
            exit(0);
        }
        delta = JointState(planner.dof(), 0);
        if (!haveToPlan)
        {
            // generating new goal
            currentState = g_tests[solved + 1].first;
            goal = g_tests[solved + 1].second;
            for (size_t i = 0; i < planner.dof(); ++i)
            {
                d->qpos[i + planner.dof()] = goal.rad(i);
                d->qpos[i] = currentState.rad(i);
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
                solution = planner.planSteps(currentState, goal, ALG_ASTAR, 600.0);
                haveToPlan = false;

                printLog(stdout, solution);
                
                if (solved == 0)
                {
                    printStatsLogHeader(logFile, solution);
                    printSceneLogHeader(scenFile, solution, currentState, goal);
                }
                printStatsLog(logFile, solution);
                printSceneLog(scenFile, solution, currentState, goal);
                ++solved;
                
                printf("solved %d/%zu\n", solved, g_tests.size());
            }
        }
    }
    else
    {
        if (partOfMove == g_unitSize - 1)
        {
            currentState += delta;
            for (size_t i = 0; i < planner.dof(); ++i)
            {
                d->qpos[i] = currentState.rad(i);
            }
            delta = solution.nextStep();
            partOfMove = 0;
        }
        else
        {
            ++partOfMove;
            for (size_t i = 0; i < planner.dof(); ++i)
            {
                d->qpos[i] += delta[i] * g_worldEps;
            }
        }
    }
}

void planner_step(mjModel* m, mjData* d, ManipulatorPlanner& planner)
{
    static int counter = 0;
    static int partOfMove = 0;
    static bool haveToPlan = false;
    static int solved = 0;
    static int notSolved = 0;

    static Solution solution;
    static JointState currentState(planner.dof(), 0);
    static JointState goal(planner.dof(), 0);
    static JointState delta(planner.dof(), 0);

    // if (solution.goalAchieved())
    {
        delta = JointState(planner.dof(), 0);
        if (!haveToPlan)
        {
            // generating new goal
            goal = randomState(planner.dof(), g_units);
            for (size_t i = 0; i < planner.dof(); ++i)
            {
                d->qpos[i + planner.dof()] = goal.rad(i);
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
                solution = planner.planSteps(currentState, goal, ALG_ASTAR, 1.0);
                haveToPlan = false;

                printLog(stdout, solution);
                
                if (solution.stats.pathVerdict == PATH_FOUND)
                {
                    if (solved == 0)
                    {
                        printStatsLogHeader(logFile, solution);
                        printSceneLogHeader(scenFile, solution, currentState, goal);
                    }
                    printStatsLog(logFile, solution);
                    printSceneLog(scenFile, solution, currentState, goal);
                    ++solved;

                    // TODO remove
                    // works if no visual
                    currentState = goal;
                    for (size_t i = 0; i < planner.dof(); ++i)
                    {
                        d->qpos[i] = currentState.rad(i);
                    }
                }
                if (solution.stats.pathVerdict == PATH_NOT_FOUND)
                {
                    ++notSolved;
                }
                
                printf("solved %d, not solved %d, time limit %.1fs\n\n", solved, notSolved, 1.0);

                if (solved + notSolved == 100)
                {
                    fclose(logFile);
                    fclose(scenFile);
                    fclose(testsFile);
                    exit(0);
                }
            }
        }
    }
    // else
    // {
    //     if (partOfMove == g_unitSize - 1)
    //     {
    //         currentState += delta;
    //         for (size_t i = 0; i < planner.dof(); ++i)
    //         {
    //             d->qpos[i] = currentState.rad(i);
    //         }
    //         delta = solution.nextStep();
    //         partOfMove = 0;
    //     }
    //     else
    //     {
    //         ++partOfMove;
    //         for (size_t i = 0; i < planner.dof(); ++i)
    //         {
    //             d->qpos[i] += delta[i] * g_worldEps;
    //         }
    //     }
    // }
}

void step(mjModel* m, mjData* d, ManipulatorPlanner& planner) {
    planner_step(m, d, planner);
    mj_step(m, d);
}

// main function
int main(int argc, const char** argv)
{
    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(modelFilename, 0, error, 1000);
    else
        if (strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // make copy for planner
    // this is really crutch and it will be work only
    // with static obstacles

    mjModel* mCopy = mj_copyModel(NULL, m);
    mjData* dCopy = mj_makeData(mCopy);

    logFile = fopen(resFilename, "w+");
    scenFile = fopen(scenFilename, "w+");
    testsFile = fopen(testsFilename, "r");

    g_tests = loadTests(testsFile);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Manipulator", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // init camera
    double arr_view[] = {90, -90, 5.5, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    srand(seed);

    // make planner
    ManipulatorPlanner planner(m->nq / 2, mCopy, dCopy);
    printf("Simulation is started!\n");

    // use the first while condition if you want to simulate for a period.
    while (!glfwWindowShouldClose(window))
    {
        //  advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        const double fps = 60.0;
        while (d->time - simstart < 1.0 / fps)
        {
            step(m, d, planner);
            if (d->ncon)
            {
                printf("Collision detected! Acc = (%f, %f)\n", fabs(d->qacc[0]), fabs(d->qacc[1]));
            }
        }

        // end go to target

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
