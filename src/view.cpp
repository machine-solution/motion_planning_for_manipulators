#include "view.h"

#include <stdio.h>

View::View() {}

void View::setUp(const mjModel* model, mjData* data)
{
    // initialize visualization data structures
    mjv_defaultCamera(&_cam);
    mjv_defaultOption(&_opt);
    mjv_defaultScene(&_scn);
    mjr_defaultContext(&_con);
    mjv_makeScene(_model, &_scn, 2000);                // space for 2000 objects
    mjr_makeContext(_model, &_con, mjFONTSCALE_150);   // model-specific context

    // init camera
    double arr_view[] = {90, -90, 5.5, 0.000000, 0.000000, 0.000000};
    _cam.azimuth = arr_view[0];
    _cam.elevation = arr_view[1];
    _cam.distance = arr_view[2];
    _cam.lookat[0] = arr_view[3];
    _cam.lookat[1] = arr_view[4];
    _cam.lookat[2] = arr_view[5];

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    _window = glfwCreateWindow(1244, 700, "Manipulator", NULL, NULL);
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    printf("Simulation is started!\n");
}
void View::step()
{
    if (!shouldClose())
    {
        show();
    }
    else
    {
        close();
    }
}

void View::close()
{
    // free visualization storage
    mjv_freeScene(&_scn);
    mjr_freeContext(&_con);
}

bool View::shouldClose()
{
    return glfwWindowShouldClose(_window);
}

void View::show()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(_window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(_model, _data, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
    mjr_render(viewport, &_scn, &_con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(_window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}
