#include "taskset.h"

#include <cstdio>
#include <stdexcept>

TaskState::TaskState(const JointState& startPos, const JointState& goalPos)
{
    _start = startPos;
    _goal = goalPos;
}

const JointState& TaskState::start() const
{
    return _start;
}
const JointState& TaskState::goal() const
{
    return _goal;
}

TaskType TaskState::type() const
{
    return TASK_STATE;
}

TaskPosition::TaskPosition(const JointState& startPos, double goalX, double goalY)
{
    _start = startPos;
    _goalX = goalX;
    _goalY = goalY;
}

const JointState& TaskPosition::start() const
{
    return _start;
}
double TaskPosition::goalX() const
{
    return _goalX;
}
double TaskPosition::goalY() const
{
    return _goalY;
}

TaskType TaskPosition::type() const
{
    return TASK_POSITION;
}

// TaskSet

TaskSet::TaskSet(size_t dof)
{
    _dof = dof;
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
    fscanf(file, "%d", &dof);
    if (dof != _dof)
    {
        throw std::runtime_error("TaskSet::loadTasks: dof in taskfile and in class are not same");
    }
    if (type == TASK_STATE)
    {
        while (!feof(file))
        {
            JointState start(dof);
            JointState goal(dof);
            for (size_t i = 0; i < dof; ++i)
            {
                fscanf(file, "%d", &start[i]);
            }
            for (size_t i = 0; i < dof; ++i)
            {
                fscanf(file, "%d", &goal[i]);
            }
            float optimal;
            fscanf(file, "%f", &optimal); // it is really unused now
            _tasks.push_back(std::make_unique<TaskState>(start, goal));
        }
    }
    else if (type == TASK_POSITION)
    {
        while (!feof(file))
        {
            JointState start(dof);
            for (size_t i = 0; i < dof; ++i)
            {
                fscanf(file, "%d", &start[i]);
            }
            double goalX, goalY;
            fscanf(file, "%f%f", &goalX, &goalY);
            float optimal;
            fscanf(file, "%f", &optimal); // it is really unused now
            _tasks.push_back(std::make_unique<TaskPosition>(start, goalX, goalY));
        }
    }
    fclose(file);
}
void TaskSet::generateRandomTasks(size_t n, TaskType type, size_t seed)
{
    srand(seed);
    if (type == TASK_STATE)
    {
        for (size_t i = 0; i < n; ++i)
        {
            _tasks.push_back(std::make_unique<TaskState>(randomState(_dof, g_units), randomState(_dof, g_units)));
        }
    }
    else if (type == TASK_POSITION)
    {
        const double bound = 2.0;
        for (size_t i = 0; i < n; ++i)
        {
            double x = (double)rand() / RAND_MAX * 2 * bound - bound;
            double y = (double)rand() / RAND_MAX * 2 * bound - bound;
            _tasks.push_back(std::make_unique<TaskPosition>(randomState(_dof, g_units), x, y));
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

