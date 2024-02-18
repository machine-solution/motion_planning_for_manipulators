#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "interactor.h"

#include <stdbool.h> 
#include <set>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

TEST_CASE("Integration test 2 dof")
{
    std::vector<double> ws = {1.0, 100.0};
    std::vector<std::string> names = {"1.0", "100.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor;
        interactor.setUp({
            "tests/integration_tests/manipulator_2.xml", // model filename
            3.0, // time
            ws[i], // w
            2, // the number of random tests
            TASK_STATE, // kind of task
            false, // random test generation
            "scenaries/test_scen.log",
            "pyplot/0/test_stats_w=" + names[i] + ".log",
            "tests/integration_tests/2-dof_example.scen",
            "pyplot/0/test_runtime_w=" + names[i] + ".log",
            "maps/c_space_0.map",
            "maps/paths_0/",
            true, // display motion
            ALG_LAZY_ASTAR, // algorithm type
            12345,
        });
        interactor.doMainLoop();
    }
}

TEST_CASE("Integration test 3 dof")
{
    std::vector<double> ws = {1.0};
    std::vector<std::string> names = {"1.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor;
        interactor.setUp({
            "tests/integration_tests/manipulator_3.xml", // model filename
            30.0, // time
            ws[i], // w
            2, // the number of random tests
            TASK_STATE, // kind of task
            true, // random test generation
            "scenaries/test_scen.log",
            "pyplot/0/test_stats_w=" + names[i] + ".log",
            "tests/integration_tests/3-dof_example.scen",
            "pyplot/0/test_runtime_w=" + names[i] + ".log",
            "maps/c_space_0.map",
            "maps/paths_0/",
            true, // display motion
            ALG_LAZY_ASTAR, // algorithm type
            12345,
        });
        interactor.doMainLoop();
    }
}
