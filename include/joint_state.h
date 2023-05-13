#pragma once

#include "global_defs.h"

#include <vector>
#include <stddef.h>
#include <initializer_list>
#include <cmath>

using std::vector;

class Action
{
public:
    Action(size_t dof = 2, int value = 0);
    Action(std::initializer_list<int> list);

    size_t dof() const;
    int operator[](size_t i) const;
    int& operator[](size_t i);
    int abs() const;

private:
    vector<int> _joints;
    size_t _dof;
};

class JointState
{
public:
    JointState(size_t dof = 2, int value = 0);
    JointState(std::initializer_list<int> list);

    int operator[](size_t i) const;
    int& operator[](size_t i);

    JointState& apply(const Action& action);
    JointState applied(const Action& action) const;

    JointState& operator=(const JointState& other);

    friend bool operator<(const JointState& state1, const JointState& state2);
    friend bool operator>(const JointState& state1, const JointState& state2);
    friend bool operator<=(const JointState& state1, const JointState& state2);
    friend bool operator>=(const JointState& state1, const JointState& state2);
    friend bool operator==(const JointState& state1, const JointState& state2);
    friend bool operator!=(const JointState& state1, const JointState& state2);

    // returns angle of i-th joint in radians [-pi, pi)
    double rad(size_t i) const;

    size_t dof() const;

    int maxJoint() const;
    int minJoint() const;

    int abs() const;

    bool isCorrect() const;

    bool hasCacheXY() const;
    double cacheX() const;
    double cacheY() const;
    void setCacheXY(double x, double y) const;

private:
    void normalize();

    vector<int> _joints;
    size_t _dof;

    mutable double _cacheX = 0;
    mutable double _cacheY = 0;
    mutable bool _hasCacheXY = false;
};

int manhattanDistance(const JointState& state1, const JointState& state2);

CostType manhattanHeuristic(const JointState& state1, const JointState& state2);

JointState randomState(size_t dof, int units = g_units);
