# Trajectory optimization package for Mini-Pupper robot
Purpose of this repository is to provide low-torque and low-impact trajectory for Mini-Pupper quadrupedal robot.

<img src="https://raw.githubusercontent.com/wiki/mayataka/mini_pupper_trajopt/images/running.gif" width="800"> 


# Requirements 
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 
- [robotoc](https://github.com/mayataka/robotoc.git)

First, install [Pinocchio](https://github.com/stack-of-tasks/pinocchio) by following the [instruction](https://stack-of-tasks.github.io/pinocchio/download.html). 
Next, install [robotoc](https://github.com/mayataka/robotoc.git) at ROBOTOC_INSTALL_DIR as

```
git clone https://github.com/mayataka/robotoc & cd robotoc
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPTIMIZE_FOR_NATIVE=ON -DCMAKE_INSTALL_PREFIX=ROBOTOC_INSTALL_DIR
make install -j4
```

and modify Python path as

```
export PYTHONPATH=ROBOTOC_INSTALL_DIR/lib/python3.8/site-packages:$PYTHONPATH 
```

e.g., write it in `~/.bashrc`.
