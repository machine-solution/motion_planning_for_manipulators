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
    virtual size_t arms() const = 0;
};

class TaskState : public ITask
{
public:
    TaskState(const MultiState& startPoses, const MultiState& goalPoses, size_t dof, size_t arms);

    const MultiState& start() const;
    const MultiState& goal() const;
    
    TaskType type() const override;
    size_t arms() const override;
private:
    MultiState _starts;
    MultiState _goals;

    size_t _dof; // dof of one manipulator
    size_t _arms; // the number of manipulators
};

class TaskPosition : public ITask
{
public:
    TaskPosition(const MultiState& startPoses, vector<double> goalXs, vector<double> goalYs, size_t dof, size_t arms);

    const MultiState& start() const;
    double goalX(size_t i) const;
    double goalY(size_t i) const;
    
    TaskType type() const override;
    size_t arms() const override;
private:
    MultiState _starts;
    vector<double> _goalXs;
    vector<double> _goalYs;

    size_t _dof; // dof of one manipulator
    size_t _arms; // the number of manipulators
};

class TaskSet
{
public:
    TaskSet(size_t dof, size_t arms);

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
    size_t _dof; // dof of one manipulator
    size_t _arms; // the number of manipulators
};
