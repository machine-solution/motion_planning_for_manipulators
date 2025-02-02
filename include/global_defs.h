#pragma once

#include <cmath>
#include <iostream>

class ManipulatorPlanner;

const int g_worldUnits = 1024; // the number of world units from [0, pi]
const int g_jumpSize = 8; // size of jump in world units

const int g_units = 64; // the number of planner units from [0, pi]
const double g_eps = (M_PI / g_units); // length of 1 planner unit
const int g_unitSize = g_worldUnits / g_units; // the number of world units in planner unit;
const int g_checkJumps = g_unitSize / g_jumpSize; // the number of jumps in check collision action, must divide g_unitSize
const double g_worldEps = (M_PI / g_worldUnits);

using CostType = float;

const CostType g_weightSmoothness = 0.0;

const CostType g_inf = 1e9;

const size_t g_maxC = 10;
