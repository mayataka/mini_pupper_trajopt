# Trajectory optimization package for Mini-Pupper robot
Purpose of this repository is to provide low-torque and low-impact trajectory for Mini-Pupper quadrupedal robot.

<img src="https://raw.githubusercontent.com/wiki/mayataka/mini_pupper_trajopt/images/running.gif" width="800"> 


## Requirements 
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 
- [robotoc](https://github.com/mayataka/robotoc.git)
- [meshcat-python](https://github.com/rdeits/meshcat-python)

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
Finally, install [meshcat-python](https://github.com/rdeits/meshcat-python) as 
```
pip install meshcat
```


## Running the trajectory optimizer
In `mini_pupper_trajopt` directory, you can run the mini-pupper's gait optimization, e.g., via `python3 running.py`.
Then the log file is generated at `mini_pupper_trajopt/rsc`.

Note: this step is totally independent from ROS and Gazebo.


## Deploying the trajectory to the mini-pupper controller
After generating the log file of the optimized trajecotory, you can install the log file via `catkin_make` in your workspace.

## Summary for the Gazebo simulation
1. Install `Pinocchio`, `robotoc`, and `meshcat-python`.
2. Run `python3 running.py` at `mini_pupper_trajopt` directory.
3. Run `catkin_make` at ROS workspace.
4. Launch Gazebo simulation as `roslaunch mini_pupper_gazebo mini_pupper.launch`