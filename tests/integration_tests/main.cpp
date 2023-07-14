#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "interactor.h"

TEST_CASE("Integration test")
{
    std::vector<double> ws = {1.0, 1.1, 1.2, 1.5, 2.0, 4.0, 10.0, 30.0, 100.0};
    std::vector<std::string> names = {"1.0", "1.1", "1.2", "1.5", "2.0", "4.0", "10.0", "30.0", "100.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor("tests/integration_tests/manipulator.xml");
        interactor.setUp({
            3.0, // time
            ws[i], // w
            10000, // the number of random tests
            TASK_POSITION, // kind of task
            false, // random test generation
            "scenaries/scen.log",
            "tests/integration_tests/pyplot/7/stats_hard_w=" + names[i] + ".log",
            "tests/integration_tests/2-dof_example.scen",
            "tests/integration_tests/pyplot/7/runtime_hard_w=" + names[i] + ".log",
            true // display motion
        });
        interactor.doMainLoop();
    }
}