#include "light_mujoco.h"

/*
This code is copied out from src mujoco and refactoring to be
faster in collision checks which we need. Use this code only
for collision checkings.
*/

bool mj_light_collision(mjModel* m, mjData* d)
{
    mj_kinematics(m, d);
    mj_collision(m, d);
    return d->ncon > 0;
}
