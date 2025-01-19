#include "interactor.h"
#include "utils.h"

#include <stdbool.h> 
#include <set>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

// main function
int main(int argc, const char** argv)
{
    vector<string> jsons = jsonFilesInDirectory("parameters/launch");
    for (const auto& pathJSON: jsons)
    {
        Interactor interactor;
        interactor.setUp(pathJSON);
        std::cerr << "Starting main loop" << std::endl;
        interactor.doMainLoop();
    }

    return 0;
}
