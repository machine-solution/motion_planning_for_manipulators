#define CATCH_CONFIG_MAIN
#include "catch2/catch_amalgamated.hpp"

#include "planner.h"
#include "astar.h"

#include <cstdio>

#include "interactor.h"

#include <stdbool.h> 
#include <set>

#include <mujoco/mujoco.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

TEST_CASE("Integration")
{
    std::vector<double> ws = {1.0};
    std::vector<std::string> names = {"1.0"};

    for (size_t i = 0; i < ws.size(); ++i)
    {
        Interactor interactor("model/2-dof/manipulator_5.xml");
        interactor.setUp({
            1.0, // time
            ws[i], // w
            10, // the number of random tests
            TASK_STATE, // kind of task
            true, // random test generation
            "scenaries/scen.log",
            "pyplot/7/stats_hard_w=" + names[i] + ".log",
            "scenaries/4_2-dof_pos_hard.scen",
            "pyplot/7/runtime_hard_w=" + names[i] + ".log",
            true // display motion
        });
        interactor.doMainLoop();
    }
    REQUIRE(true);
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

// test empty plane scenario
void testPlanningFromTo(JointState a, JointState b, int alg)
{
    REQUIRE(a.dof() == b.dof());
    ManipulatorPlanner planner(a.dof());
    Solution solution = planner.planActions(a, b, alg);
    while (!solution.goalAchieved())
    {
        a.apply(solution.nextAction());
    }
    REQUIRE(a == b);
}

// test row of empty plane scenarios
void testStressPlanning(int dof, int alg)
{
    srand(57283);
    ManipulatorPlanner planner(dof);
    JointState a(dof, 0);
    JointState b = randomState(dof);
    for (size_t i = 0; i < 20; ++i)
    {
        Solution solution = planner.planActions(a, b, alg);
        while (!solution.goalAchieved())
        {
            a.apply(solution.nextAction());
        }
        REQUIRE(a == b);
        b = randomState(dof);
    }
}

TEST_CASE("Linear planner on empty plane")
{
    testPlanningFromTo({0, 0}, {0, 0}, ALG_LINEAR);
    testPlanningFromTo({0, 0}, {1, 1}, ALG_LINEAR);
    testPlanningFromTo({1, 2}, {3, -4}, ALG_LINEAR);
    testPlanningFromTo({5, -3}, {-1, -3}, ALG_LINEAR);
    testPlanningFromTo({5}, {-1}, ALG_LINEAR);
    testPlanningFromTo({5, 5, 5}, {-1, -1, -1}, ALG_LINEAR);
    testPlanningFromTo({5, 5, 5, 5}, {-1, -1, -1, -1}, ALG_LINEAR);
    testStressPlanning(1, ALG_LINEAR);
    testStressPlanning(2, ALG_LINEAR);
    testStressPlanning(3, ALG_LINEAR);
    testStressPlanning(4, ALG_LINEAR);
}

TEST_CASE("A* planner on empty plane")
{
    testPlanningFromTo({0, 0}, {0, 0}, ALG_ASTAR);
    testPlanningFromTo({0, 0}, {1, 1}, ALG_ASTAR);
    testPlanningFromTo({1, 2}, {3, -4}, ALG_ASTAR);
    testPlanningFromTo({5, -3}, {-1, -3}, ALG_ASTAR);
    testPlanningFromTo({5}, {-1}, ALG_ASTAR);
    testPlanningFromTo({5, 5, 5}, {-1, -1, -1}, ALG_ASTAR);
    testPlanningFromTo({5, 5, 5, 5}, {-1, -1, -1, -1}, ALG_ASTAR);
    testStressPlanning(1, ALG_ASTAR);
    testStressPlanning(2, ALG_ASTAR);
    testStressPlanning(3, ALG_ASTAR);
    testStressPlanning(4, ALG_ASTAR);
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
    REQUIRE(tree.size() == 4);
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
