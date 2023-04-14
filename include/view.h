#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

class View
{
public:
    View();

    void setUp(const mjModel* model, mjData* data);
    void step();
    void close();
    bool shouldClose();
    void show();

private:
    // mujoco data structures
    const mjModel* _model;
    mjData* _data;      // it used in updateScene like not const pointer
    mjvCamera _cam;     // abstract camera
    mjvOption _opt;     // visualization options
    mjvScene _scn;      // abstract scene
    mjrContext _con;    // custom GPU context

    // main window
    GLFWwindow* _window;
};
