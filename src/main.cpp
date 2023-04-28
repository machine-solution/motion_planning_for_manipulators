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
    // std::vector<double> ws = {1.0, 1.1, 1.2, 1.5, 2.0, 4.0, 10.0, 30.0, 100.0};
    // std::vector<std::string> names = {"1.0", "1.1", "1.2", "1.5", "2.0", "4.0", "10.0", "30.0", "100.0"};

    std::vector<double> ws = {1.0};
    std::vector<std::string> names = {"1.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor("model/2-dof/manipulator_4.xml");
        interactor.setUp({
            3.0, // time
            ws[i], // w
            1000,
            TASK_POSITION,
            false, // random test generation
            "scenaries/scen.log",
            "pyplot/6/stats_dof=2_w=" + names[i] + ".log",
            "scenaries/4_2-dof_pos_hard.scen",
            true // display motion
        });
        interactor.doMainLoop();
    }

    // for (size_t i = 0; i < ws.size(); ++i)
    // {
    //     Interactor interactor("model/4-dof/manipulator_4.xml");
    //     interactor.setUp({
    //         1.0, // time
    //         ws[i], // w
    //         1000,
    //         false,
    //         "scenaries/4_4-dof_scen.log",
    //         "pyplot/5/stats_dof=4_w=" + names[i] + ".log",
    //         "scenaries/4_4-dof_hard.scen"
    //     });
    //     interactor.doMainLoop();
    // }

    return 0;
}
