#pragma once

#include <vector>
#include <stddef.h>
#include <initializer_list>
#include <cmath>

using std::vector;

const int g_units = 1024; // the number of units from [0, pi]
const double g_eps = (M_PI / g_units); // length of 1 unit

class JointState
{
public:
    JointState(size_t dof = 2, int value = 0);
    JointState(std::initializer_list<int> list);

    int operator[](size_t i) const;
    int& operator[](size_t i);

    JointState operator+(const JointState& other) const;

    JointState& operator=(const JointState& other);
    JointState& operator+=(const JointState& other);

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

    bool isCorrect() const;

    // const int units = g_units;
    // const double eps = g_eps;

private:
    void normalize();

    vector<int> _joints;
    size_t _dof;
};

int manhattanDistance(const JointState& state1, const JointState& state2);

JointState randomState(size_t dof, int units = g_units);
