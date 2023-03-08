This project uses [mujoco](https://github.com/deepmind/mujoco) as simulator and library.  

# Installation
This instruction is written for linux. Before start make sure that you have g++ compiler installed on your machine.  
To compile and run this project you need to install OpenGL (glfw) library

```
sudo apt update
sudo apt install libglfw3-dev
```

Then copy-paste two files from 'lib' folder libmujoco.so and libmujoco.so.2.3.2 to '/usr/lib'.

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
make
```

After this operation in the root of this repository you can find 'manipulator' file. Run it.  
```
./manipulator
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
