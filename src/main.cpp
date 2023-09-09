#include "interactor.h"

#include <stdbool.h> 
#include <set>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

// main function
int main(int argc, const char** argv)
{
    std::vector<double> ws = {1.0, 1.1, 1.2, 1.5, 2.0, 4.0, 10.0, 30.0, 100.0};
    std::vector<std::string> names = {"1.0", "1.1", "1.2", "1.5", "2.0", "4.0", "10.0", "30.0", "100.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor("model/2-dof/manipulator_5.xml");
        interactor.setUp({
            3.0, // time
            ws[i], // w
            10000, // the number of random tests
            TASK_STATE, // kind of task
            true, // random test generation
            "scenaries/scen.log", // output for data about tasks
            "pyplot/7/stats_hard_w=" + names[i] + ".log", // algorithm stats output
            "scenaries/4_2-dof_pos_hard.scen", // input tasks list
            "pyplot/7/runtime_hard_w=" + names[i] + ".log", // profiling output
            true // display motion
        });
        interactor.doMainLoop();
    }

    return 0;
}
