# New kind of tasks

## Problem definition
We have start position and final coordinates of end-effector. Problem in planning path to reach end-effector final position.
It is certain that we cannot exactly reach end position for any position because we discretized configuration space. In own worl problem in planning path to reach position of end-effector far not more than eps (now eps = 10^(-6)) from final position.

## Step 1 : it should works at least somehow

### What I do for step 1
I needed some abilities in this project to make this problem definition.
1. I need to control in A* condition "current position is goal", because previous 'if (current position == goal position)' already isn't works. I made isGoal method in AstarChecker (AstarChecker is class with methods A* use for planning. It helps configurate some steps in A* and not rewrite main A* logic).
2. Create heuristic for new problem definition. I calculated maximum distance whitch end-effector can overcome in one step and in heuristic function I devide distance from current end-effector and goal by this maximum end-effector step.
3. Generating tasks. Isn't closed question, but for first time I generate tasks by whole pos and use only end-effector position. This method gives garantees that any task will be solvable. Another advantage in ability to reuse already clusterized tests in the same format. But in future I need to generate task 'honestly'.
4. For new task create new methods, models (without green goal pos) and etc. This work I didn't make (because paragraph 3), but it can create more difficulties in future and needs much time.

### Comparation
I compared new A* tasks with old on old hard testset. There is one problem: a lot of test hard for the first problem definition isn't hard for the second in this map. In other hand I thought there are very difficult test in this statement because not best heuristic and set time budget 10 seconds for every weight.\
![](pictures/5/runtime_small_w.png)\
This plot shows how increased runtime in new task definition. There are two reasons: increased number of expansions and more slow calculations. To calculate end-effector pos I call mujoco function 'mj_forward'. May be I can rewrite light function for my situation.\
![](pictures/5/suboptimality_small_w.png)\
But in this plot we can see, that small weights give optimal solutions. Next plot show that only weight=4.0 and more give not optimal solutions.\
![](pictures/5/suboptimality_all_w.png)\
As we can see on next two plots new task definition with big wieght make almost same perfomance as old task. And the number of expansions are less. But it isn't achievement of algorithm - many of 'hard' for first problem tasks aren't 'hard' for second.\
![](pictures/5/runtime_all_w.png)\
![](pictures/5/expansions_all_w.png)\
At the end I say that one expansion takes more time in new task. I beleive I can improve it using low level optimizations.

### Conclusion step 1
1. New task works more slow and need speeding up by high and low level optimizations.
2. New task hard to implement in one step and I will continue integrating the second variant of test to project.

## Step 2 : divide two types of tests

### New task in code
In previous step I said that generating, reading and writing tests of new type not implemented yet. In this section I describe how I implemented their. In the last version class there was only one class Task. Now I add classes TaskState (old problem) and TaskPosition (new problem). They are inherited from ITask abstract class and stored in TaskSet like vector of ITask* (ITask pointers). Now in every method I check type of test (ITask has abstract method ITask::type()) and use one of two planSteps method of planner. Methods differ only in arguments.

### New task generation
In taskset I implemented new random task generation. It generates random start state and random end-effector goal in [-1,1]^2 square. But in first run algorithm solved 0 tasks from 500. Problem was in determining the target state. At the previous step I determine goal as state in which end-effector far not more than eps (now eps = 10^(-6)) from final position. It worked fine in tests where exists state in which end-effector placed exactly on final position. But for random positions it isn't good idea. I have change 10^(-6) to 0.05 and task generation system and task solving began working correctly. This task generator (as for previous problem) can generate incorrect tasks. It isn't good.

### Read and write tests
I want to read old tasksets without any changes, but also I want to read any taskset without difference in methods and arguments. It is obviously, that I can't achieve both results at the same time and I chose reading tasksets without changes and now in config I must choose task types to read files or generating it at random. In this scenario now I can read and write test using Logger. Logger writes tests log and small python script reads it and rewrite to taskset format.

### Conclusions step 2
1. Now in project I can use both types of tasks but read and generate only one type in one interactor.
2. There is a not good task generator in this project and I need to improve it in future to generate more correct tasks.
