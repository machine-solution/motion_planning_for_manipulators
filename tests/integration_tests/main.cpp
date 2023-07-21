#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "interactor.h"
#include <mujoco/mujoco.h>

TEST_CASE("Integration test")
{
    Interactor interactor("tests/integration_tests/manipulator.xml");
    interactor.setUp({
        3.0, // time
        1.0, // w
        10000, // the number of random tests
        TASK_POSITION, // kind of task
        false, // random test generation
        "scenaries/scen.log",
        "tests/integration_tests/pyplot/7/stats_hard_w=1.0.log",
        "tests/integration_tests/2-dof_example.scen",
        "tests/integration_tests/pyplot/7/runtime_hard_w=1.0.log",
        true // display motion
    });
    interactor.doMainLoop();
}