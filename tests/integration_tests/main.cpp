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

TEST_CASE("Integration test")
{
    std::vector<double> ws = {1.0, 100.0};
    std::vector<std::string> names = {"1.0", "100.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor("tests/integration_tests/manipulator.xml");
        interactor.setUp({
            3.0, // time
            ws[i], // w
            2, // the number of random tests
            TASK_STATE, // kind of task
            false, // random test generation
            "scenaries/scen.log",
            "pyplot/7/stats_hard_w=" + names[i] + ".log",
            "tests/integration_tests/2-dof_example.scen",
            "pyplot/7/runtime_hard_w=" + names[i] + ".log",
            true // display motion
        });
        interactor.doMainLoop();
    }
}
