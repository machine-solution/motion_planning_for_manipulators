#include "taskset.h"

#include <iostream>
#include <cstdio>
#include <stdexcept>

TaskState::TaskState(const MultiState &startPoses, const MultiState &goalPoses, size_t dof, size_t arms)
{
    _starts = startPoses;
    _goals = goalPoses;
    _arms = arms;
    _dof = dof;
}

const MultiState& TaskState::start() const
{
    return _starts;
}
const MultiState& TaskState::goal() const
{
    return _goals;
}

TaskType TaskState::type() const
{
    return TASK_STATE;
}

size_t TaskState::arms() const
{
    return _arms;
}

TaskPosition::TaskPosition(const MultiState& startPoses, vector<double> goalXs, vector<double> goalYs, size_t dof, size_t arms)
{
    _starts = startPoses;
    _goalXs = goalXs;
    _goalYs = goalYs;
    _arms = arms;
    _dof = dof;
}

const MultiState& TaskPosition::start() const
{
    return _starts;
}
double TaskPosition::goalX(size_t i) const
{
    return _goalXs.at(i);
}
double TaskPosition::goalY(size_t i) const
{
    return _goalYs.at(i);
}

TaskType TaskPosition::type() const
{
    return TASK_POSITION;
}

size_t TaskPosition::arms() const
{
    return _arms;
}

// TaskSet

TaskSet::TaskSet(size_t dof, size_t arms)
{
    _dof = dof;
    _arms = arms;
    _nextTaskId = 0;
}

void TaskSet::loadTasks(const std::string& filename, TaskType type)
{
    FILE* file = fopen(filename.c_str(), "r");
    if (file == nullptr)
    {
        throw std::runtime_error("TaskSet::loadTasks: Could not open file " + filename);
    }
    int dof;
    int arms;
    int scanned = fscanf(file, "%d %d", &dof, &arms);
    if (scanned != 2)
    {
        throw std::runtime_error("TaskSet::loadTasks: dof or/and arms was not scanned");
    }
    if (dof != _dof)
    {
        throw std::runtime_error("TaskSet::loadTasks: dof in taskfile and in class are not same");
    }
    if (arms != _arms)
    {
        throw std::runtime_error("TaskSet::loadTasks: arms in taskfile and in class are not same");
    }
    if (type == TASK_STATE)
    {
        while (!feof(file))
        {
            std::vector<JointState> starts(arms, JointState(dof));
            std::vector<JointState> goals(arms, JointState(dof));
            int counter_scanned = 0;
            for (size_t a = 0; a < arms; ++a)
            {
                for (size_t i = 0; i < dof; ++i)
                {
                    counter_scanned += fscanf(file, "%d", &starts[a][i]);
                }
            }
            for (size_t a = 0; a < arms; ++a)
            {
                for (size_t i = 0; i < dof; ++i)
                {
                    counter_scanned += fscanf(file, "%d", &goals[a][i]);
                }
            }
            if (counter_scanned == (2 * dof * arms))
            {
                _tasks.push_back(std::make_unique<TaskState>(MultiState(starts), MultiState(goals), dof, arms));
            }
            else
            {
                break;
            }
        }
    }
    fclose(file);
}
void TaskSet::generateRandomTasks(size_t n, TaskType type, const ManipulatorPlanner& planner, size_t seed)
{
    srand(seed);
    if (type == TASK_STATE)
    {
        size_t created_tasks = 0;
        while (created_tasks < n)
        {
            MultiState starts(_dof, _arms);
            MultiState ends(_dof, _arms);
            for (size_t a = 0; a < _arms; ++a)
            {
                starts[a] = randomState(_dof, g_units);
                ends[a] = randomState(_dof, g_units);
            }
            if (!planner.checkMultiCollision(starts) && !planner.checkMultiCollision(ends))
            {
                _tasks.push_back(std::make_unique<TaskState>(starts, ends, _dof, _arms));
                ++created_tasks;
                std::cout << "  CORRECT TASK " << created_tasks << "/" << n << std::endl;
            }
            else
            {
                std::cout << "INCORRECT TASK " << created_tasks << "/" << n << std::endl;
            }
        }
    }
    else if (type == TASK_POSITION)
    {
        const double bound = 2.0;
        size_t created_tasks = 0;
        while (created_tasks < n)
        {
            MultiState starts;
            std::vector<double> xs(_arms), ys(_arms);
            for (size_t a = 0; a < _arms; ++a)
            {
                xs[a] = (double)rand() / RAND_MAX * 2 * bound - bound;
                ys[a] = (double)rand() / RAND_MAX * 2 * bound - bound;
                starts[a] = randomState(_dof, g_units);
            }
            if (!planner.checkMultiCollision(starts))
            {
                _tasks.push_back(std::make_unique<TaskPosition>(starts, xs, ys, _dof, _arms));
                ++created_tasks;
            }
        }
    }
}
void TaskSet::removeTasks()
{
    _tasks.clear();
    _nextTaskId = 0;
}
void TaskSet::restartTasks()
{
    _nextTaskId = 0;
}

const ITask* TaskSet::getTask(size_t i) const
{
    return _tasks[i].get();
}
const ITask* TaskSet::getNextTask()
{
    return _tasks[_nextTaskId++].get();
}
bool TaskSet::haveNextTask() const
{
    return _nextTaskId < _tasks.size();
}

size_t TaskSet::progress() const
{
    return _nextTaskId;
}
size_t TaskSet::size() const
{
    return _tasks.size();
}

