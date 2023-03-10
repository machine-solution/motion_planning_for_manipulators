#define CATCH_CONFIG_MAIN
#include "catch2/catch_amalgamated.hpp"

#include "planner.h"
#include "astar.h"

#include <cstdio>

TEST_CASE("JointState arithmetic")
{
    JointState a({1, 2});
    JointState b({3, -4});
    a += b;
    REQUIRE(a == JointState({4, -2}));
    b += JointState({1, 0});
    REQUIRE(b == JointState({4, -4}));
    REQUIRE(a != b);
    REQUIRE(JointState{4, 0} + JointState{0, -4} == b);
    b += a;
    REQUIRE(JointState{4, -4} + a == b);
}

TEST_CASE("JointState comparation")
{
    JointState a({1, 2});
    JointState b({3, -4});
    JointState c({5});
    REQUIRE(a < b);
    REQUIRE(c < a);
    REQUIRE(c < b);
    REQUIRE(a > c);
    REQUIRE(a <= b);
    REQUIRE(c != a);
    REQUIRE(b >= c);
    REQUIRE(b >= JointState({2, 2}));
    REQUIRE(c == JointState({5}));
}

TEST_CASE("Linear planner on empty plane")
{
    ManipulatorPlanner planner(2);
    JointState a({1, 2});
    JointState b({3, -4});
    planner.planSteps(a, b, ALG_LINEAR);
    while (!planner.goalAchieved())
    {
        a += planner.nextStep();
    }
    REQUIRE(a == b);
}

// test empty plane scenarioo
void testPlanningFromTo(JointState a, JointState b, int alg)
{
    ManipulatorPlanner planner(2);
    planner.planSteps(a, b, alg);
    while (!planner.goalAchieved())
    {
        a += planner.nextStep();
    }
    REQUIRE(a == b);
}

TEST_CASE("A* planner on empty plane")
{
    testPlanningFromTo({0, 0}, {0, 0}, ALG_ASTAR);
    testPlanningFromTo({0, 0}, {1, 1}, ALG_ASTAR);
    testPlanningFromTo({1, 2}, {3, -4}, ALG_ASTAR);
    testPlanningFromTo({5, -3}, {-1, -3}, ALG_ASTAR);
}

TEST_CASE("A* Nodes has operators")
{
    astar::SearchNode node1(1, 0, JointState(1, 0));
    astar::SearchNode node2(2, 0, JointState(1, 0));
    REQUIRE(node1 < node2);
    astar::SearchNode* p1 = &node1;
    astar::SearchNode* p2 = &node2;
    REQUIRE(p1 < p2);
}

TEST_CASE("A* Search Tree")
{
    astar::SearchTree tree;
    astar::SearchNode* n1 = new astar::SearchNode(1, 0, JointState({1, 0})); // same state as 2
    astar::SearchNode* n2 = new astar::SearchNode(2, 0, JointState({1, 0}));
    astar::SearchNode* n3 = new astar::SearchNode(1, 0, JointState({1, 1})); // same state as 4
    astar::SearchNode* n4 = new astar::SearchNode(2, 0, JointState({1, 1}));
    tree.addToOpen(n1);
    tree.addToOpen(n2);
    tree.addToOpen(n3);
    tree.addToOpen(n4);
    astar::SearchNode* best = tree.extractBestNode();
    REQUIRE(best != nullptr);
    REQUIRE(best->f() == 1);
    tree.addToClosed(best);
    best = tree.extractBestNode();
    REQUIRE(best != nullptr);
    REQUIRE(best->f() == 1);
    tree.addToClosed(best);
    best = tree.extractBestNode();
    REQUIRE(best == nullptr);
}
