This project uses [mujoco](https://github.com/deepmind/mujoco) as simulator and library.

Content
1. [Installation](#installation)
1. [Run](#run)
1. [Run tests](#run-tests)
1. [Project description](#project-description)
    1. [Problem description](#problem-description)
    1. [Planner](#planner)
        1. [General description](#general-description)
        1. [A* algorithm](#a-algorithm)
        1. [Collision checking](#collision-checking)
    1. [Task generation](#task-generation)
    1. [Model](#model)
    1. [Interaction with mujoco](#interaction-with-mujoco)

# Installation
This instruction is written for linux. Before start make sure that you have g++ compiler installed on your machine.  
To compile and run this project you need to install OpenGL (glfw) library

```
sudo apt update
sudo apt install libglfw3-dev
```

Then copy-paste two files from 'lib' folder libmujoco.so and libmujoco.so.2.3.2 to '/usr/lib' to install mujoco libraries

```
sudo cp lib/libmujoco.so /usr/lib/libmujoco.so
sudo cp lib/libmujoco.so.2.3.2 /usr/lib/libmujoco.so.2.3.2
```
Congradulations! You have completed installation.  

# Run
For run on linux you can use 'run_linux' file. Execute this in the root of the repository.  
```
./run_linux
```

Or you can compile this using Makefile
```
make simulator
```

After this operation in the root of this repository you can find 'manipulator' file. Run it.  
```
./simulator
```

If you get into trouble with running some of this files, do this and try again:
```
chmod +x <execution_file>
```

# Run tests
For run tests you just need to execute this file.
```
./run_tests
```
First run may be slow.

# Project description

## Problem description
We have a manipulator in space with obstacles and two positions of this manipulator: start and finish. It is required to plan the actions using which the manipulator will come from the start position to the finish.

## Planner

### General description
The most important class in this project is [ManipulatorPlanner](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/include/planner.h#L16). It solves problem in method [planSteps](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/include/planner.h#L30). This method must return solution with statistics and sequence of actions needed to reach finish from start.

### A* algorithm
A* algorithm is realized in two places: [node and tree](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/astar.cpp#L8) and [algorithm](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/planner.cpp#L183) in planner.\
It is planned to move algorithm to astar.cpp.

### Collision checking
For collision checking I use copy of original model on scene. Planner [gets this copy](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/main.cpp#L345) and uses it in [checkCollisionAction](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/planner.cpp#L36) and [checkCollision](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/planner.cpp#L22) methods.\
For speed I use [light_collision](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/light_mujoco.cpp#L96) function instead mujoco standard 'mj_step_1'. Code of this function was copied from mujoco source files and refactored to more light function. But it has one constraint: it works only for predefined pairs of geoms. It means that you have to define in model file witch pair of geoms we need to check on collision. It makes some of discomfort, but gains about 20% speeding up.

## Task generation
Now task generation is united with interactor and use only one function - [randomState](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/main.cpp#L215). This process very simple: manipulator strts with zero state and gets random finish. If it finds solution, it will get new random finish and start will be previous finish. If no, it gets new random finish without changing start.\
I'm planning to separate interaction and test generation in future.

## Model
For this project we can do manipulator model in xml format. To learn how to do this please read [xml reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html).\
All models are located in 'model' directory.\
Now the easiest way to change model is execute   ```./simulator <filename>```   where filename is path to your model from root of repository.\
Another way to do this is change default filename [here](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/main.cpp#L14).

## Interaction with mujoco
All interaction planner and mujoco simulator in [planner_step](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/src/main.cpp#L197) function, which is being called in infinity loop of simulation.\
To simulate actions of manipulator I divided angles from 0 to pi on [worldUnits](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/include/global_defs.h#L8) - minimal angle to move.\
Planner can divide angles on another [units](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/include/global_defs.h#L5) and when interaction function gets next step, it move manipulator in step direction, but every interactor step move manipulator on one worlUnit and do this [unitSize](https://github.com/machine-solution/motion_planning_for_manipulators/blob/261f3460d69ccef7a86ff90b380b45a91f1aa76f/include/global_defs.h#L7) times.
