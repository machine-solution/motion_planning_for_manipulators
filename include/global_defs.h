#pragma once

#include <cmath>

const int g_units = 32; // the number of planner units from [0, pi]
const double g_eps = (M_PI / g_units); // length of 1 planner unit
const int g_unitSize = 32; // the number of world units in planner unit;
const int g_checkJumps = 1; // the number of jumps in check collision action, must divide g_unitSize
const int g_worldUnits = g_units * g_unitSize;
const double g_worldEps = (M_PI / g_worldUnits);

using CostType = float;

const CostType g_weightSmoothness = 0.0;
