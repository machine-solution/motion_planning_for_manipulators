#pragma once

#include "joint_state.h"
#include "planner.h"

#include <string>
#include <memory>

enum TaskType
{
    TASK_STATE,
    TASK_POSITION,
    TASK_MAX // the number of Task types
};

class ITask
{
public:
    virtual TaskType type() const = 0;
};

class TaskState : public ITask
{
public:
    TaskState(const JointState& startPos, const JointState& goalPos);

    const JointState& start() const;
    const JointState& goal() const;
    
    TaskType type() const override;
private:
    JointState _start;
    JointState _goal;
};

class TaskPosition : public ITask
{
public:
    TaskPosition(const JointState& startPos, double goalX, double goalY);

    const JointState& start() const;
    double goalX() const;
    double goalY() const;
    
    TaskType type() const override;
private:
    JointState _start;
    double _goalX;
    double _goalY;
};

class TaskSet
{
public:
    TaskSet(size_t dof);

    void loadTasks(const std::string& filename, TaskType type);
    void generateRandomTasks(size_t n, TaskType type, const ManipulatorPlanner& planner, size_t seed = 12345);
    void removeTasks();
    void restartTasks();

    const ITask* getTask(size_t i) const;
    const ITask* getNextTask();
    bool haveNextTask() const;

    size_t progress() const;
    size_t size() const;

private:
    std::vector<std::unique_ptr<ITask>> _tasks;
    size_t _nextTaskId;
    size_t _dof;
};
