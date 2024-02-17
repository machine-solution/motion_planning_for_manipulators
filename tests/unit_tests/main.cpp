#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "astar.h"
#include "planner.h"
#include "taskset.h"
#include "utils.h"

#include <cstdio>

TEST_CASE("JointState comparation")
{
    JointState a({1, 2});
    JointState b({3, -4});
    JointState c({5});
    CHECK(a < b);
    CHECK(c < a);
    CHECK(c < b);
    CHECK(a > c);
    CHECK(a <= b);
    CHECK(c != a);
    CHECK(b >= c);
    CHECK(b >= JointState({2, 2}));
    CHECK(c == JointState({5}));
}

// test empty plane scenario
void testPlanningFromTo(JointState a, JointState b, int alg)
{
    CHECK(a.dof() == b.dof());
    ManipulatorPlanner planner(a.dof());
    Solution solution = planner.planActions(a, b, alg);
    while (!solution.goalAchieved())
    {
        a.apply(solution.nextAction());
    }
    CHECK(a == b);
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
        CHECK(a == b);
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

TEST_CASE("lazy A* planner on empty plane")
{
    testPlanningFromTo({0, 0}, {0, 0}, ALG_LAZY_ASTAR);
    testPlanningFromTo({0, 0}, {1, 1}, ALG_LAZY_ASTAR);
    testPlanningFromTo({1, 2}, {3, -4}, ALG_LAZY_ASTAR);
    testPlanningFromTo({5, -3}, {-1, -3}, ALG_LAZY_ASTAR);
    testPlanningFromTo({5}, {-1}, ALG_LAZY_ASTAR);
    testPlanningFromTo({5, 5, 5}, {-1, -1, -1}, ALG_LAZY_ASTAR);
    testPlanningFromTo({5, 5, 5, 5}, {-1, -1, -1, -1}, ALG_LAZY_ASTAR);
    testStressPlanning(1, ALG_LAZY_ASTAR);
    testStressPlanning(2, ALG_LAZY_ASTAR);
    testStressPlanning(3, ALG_LAZY_ASTAR);
    testStressPlanning(4, ALG_LAZY_ASTAR);
}

TEST_CASE("A* Nodes has operators")
{
    astar::SearchNode node1(1, 0, JointState(1, 0));
    astar::SearchNode node2(2, 0, JointState(1, 0));
    CHECK(node1 < node2);
    astar::SearchNode* p1 = &node1;
    astar::SearchNode* p2 = &node2;
    CHECK(p1 < p2);
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
    CHECK(tree.size() == 4);
    astar::SearchNode* best = tree.extractBestNode();
    CHECK(best != nullptr);
    CHECK(best->f() == 1);
    tree.addToClosed(best);
    best = tree.extractBestNode();
    CHECK(best != nullptr);
    CHECK(best->f() == 1);
    tree.addToClosed(best);
    best = tree.extractBestNode();
    CHECK(best == nullptr);
}

void testReadFile(int dof, const std::string& file_path, int number_of_tests, TaskType type)
{
    TaskSet *taskset = new TaskSet(dof);
    taskset->loadTasks(file_path, type);
    CHECK(taskset->size() == number_of_tests);
}

TEST_CASE("File read test")
{
    testReadFile(2, "tests/unit_tests/samples/load_taskset/2-dof_pos_test_1.scen", 0, TASK_POSITION);
    testReadFile(2, "tests/unit_tests/samples/load_taskset/2-dof_test_2.scen", 4, TASK_STATE);
    testReadFile(2, "tests/unit_tests/samples/load_taskset/2-dof_pos_test_3.scen", 2, TASK_POSITION);
    testReadFile(3, "tests/unit_tests/samples/load_taskset/3-dof_test_4.scen", 4, TASK_STATE);
}

TEST_CASE("Recursively search files in directory")
{
    std::vector<std::string> answer{
        "tests/unit_tests/samples/dir/file1.txt",
        "tests/unit_tests/samples/dir/file2.txt",
        "tests/unit_tests/samples/dir/subdir",
        "tests/unit_tests/samples/dir/subdir/subfile1.txt",
        "tests/unit_tests/samples/dir/subdir/subjson.json",
    };
    std::vector<std::string> result = filesInDirectory("tests/unit_tests/samples/dir");
    CHECK(answer == result);
}

TEST_CASE("Recursively search json files in directory")
{
    std::vector<std::string> answer{
        "tests/unit_tests/samples/dir/subdir/subjson.json",
    };
    std::vector<std::string> result = jsonFilesInDirectory("tests/unit_tests/samples/dir");
    CHECK(answer == result);
}

TEST_CASE("Planner primitive actions with opposite indexes are reversed")
{
    int dof = 4;
    ManipulatorPlanner planner(dof);
    const vector<Action>& actions = planner.getPrimitiveActions();
    printf("size %zu\n", actions.size());
    for (int i = 0; i < actions.size(); ++i)
    {
        Action current = actions[i];
        Action opposite = actions[actions.size() - i - 1];
        bool reversed = true;
        for (int j = 0; j < dof; ++j)
        {
            reversed &= current[j] == -opposite[j];
        }
        CHECK(reversed);
    }
}

// test empty plane scenario
void testReversePlanningFromTo(JointState a, JointState b, int alg)
{
    CHECK(a.dof() == b.dof());
    ManipulatorPlanner planner(a.dof());
    Solution solution = planner.planActions(b, a, alg).reversed();
    while (!solution.goalAchieved())
    {
        a.apply(solution.nextAction());
    }
    CHECK(a == b);
}

// test row of empty plane scenarios
void testStressReversePlanning(int dof, int alg)
{
    srand(57283);
    ManipulatorPlanner planner(dof);
    JointState a(dof, 0);
    JointState b = randomState(dof);
    for (size_t i = 0; i < 20; ++i)
    {
        Solution solution = planner.planActions(b, a, alg).reversed();
        while (!solution.goalAchieved())
        {
            a.apply(solution.nextAction());
        }
        CHECK(a == b);
        b = randomState(dof);
    }
}

TEST_CASE("Planner reversed solution on empty plane")
{
    testReversePlanningFromTo({0, 0}, {0, 0}, ALG_LINEAR);
    testReversePlanningFromTo({0, 0}, {1, 1}, ALG_ASTAR);
    testReversePlanningFromTo({1, 2}, {3, -4}, ALG_LAZY_ASTAR);
    testReversePlanningFromTo({5, -3}, {-1, -3}, ALG_LINEAR);
    testReversePlanningFromTo({5}, {-1}, ALG_ASTAR);
    testReversePlanningFromTo({5, 5, 5}, {-1, -1, -1}, ALG_LAZY_ASTAR);
    testReversePlanningFromTo({5, 5, 5, 5}, {-1, -1, -1, -1}, ALG_LINEAR);
    testStressReversePlanning(1, ALG_ASTAR);
    testStressReversePlanning(2, ALG_LAZY_ASTAR);
    testStressReversePlanning(3, ALG_LINEAR);
    testStressReversePlanning(4, ALG_ASTAR);
}
