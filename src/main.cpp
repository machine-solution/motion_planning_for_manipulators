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
    interactor.setUp();

    interactor.doMainLoop();

    return 0;
}
