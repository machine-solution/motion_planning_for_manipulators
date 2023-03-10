#define CATCH_CONFIG_MAIN
#include "catch2/catch_amalgamated.hpp"

#include "planner.h"

TEST_CASE("JointState arithmetic")
{
    JointState a({1, 2});
    JointState b({3, -4});
    a += b;
    REQUIRE(a == JointState({4, -2}));
    b += JointState({1, 0});
    REQUIRE(b == JointState({4, -4}));
    REQUIRE(a != b);
}

TEST_CASE("Planner on empty plane")
{
    ManipulatorPlanner planner(2);
    JointState a({1, 2});
    JointState b({3, -4});
    planner.planSteps(a, b);
    while (!planner.goalAchieved())
    {
        a += planner.nextStep();
    }
    REQUIRE(a == b);
}
