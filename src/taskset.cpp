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

TaskSet::TaskSet(size_t dof)
{
    _dof = dof;
    _nextTaskId = 0;
}
TaskSet::TaskSet(size_t dof, const std::string& filename) : TaskSet(dof)
{
    loadTasks(filename);
}
TaskSet::TaskSet(size_t dof, size_t n, size_t seed) : TaskSet(dof)
{
    generateRandomTasks(n, seed);
}

void TaskSet::loadTasks(const std::string& filename)
{
    FILE* file = fopen(filename.c_str(), "r");
    // This may be called in constructor and
    // exceptions in constructor is a bad idea
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
    fclose(file);
}
void TaskSet::generateRandomTasks(size_t n, size_t seed)
{
    srand(seed);
    for (size_t i = 0; i < n; ++i)
    {
        _tasks.push_back(std::make_unique<TaskState>(randomState(_dof, g_units), randomState(_dof, g_units)));
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

