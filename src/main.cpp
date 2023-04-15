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
    Interactor interactor("model/2-dof/manipulator_4.xml");
    interactor.setUp({
        10.0, // time
        10.0, // w
        100,
        true,
        "scenaries/scen.log",
        "pyplot/4/stats.log",
        "scenaries/4_hard.scen"
    });

    interactor.doMainLoop();

    return 0;
}
