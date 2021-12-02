# Trajectory optimization package for Mini-Pupper robot
The purpose of this repository is to provide low-torque and low-impact trajectories for Mini-Pupper quadrupedal robot.

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
In `mini_pupper_trajopt` directory, you can run the mini-pupper's gait optimization, e.g., via `python3 troting.py`.
Then the log file is generated at `mini_pupper_trajopt/rsc`, e.g., `mini_pupper_trajopt/rsc/trotting/q.log`.

<strong>Note</strong>: this step is totally independent of ROS and Gazebo.


## Deploying the trajectory to the mini-pupper controller
After generating the log file of the optimized trajectory, you can install the log file via `catkin_make` in your workspace.
That is, `catkin_make` command also installs `mini_pupper_trajopt/rsc/trotting/q.log`.
You can then run a Gazebo simulation as `roslaunch mini_pupper_gazebo mini_pupper.launch`.


<strong>Remark</strong>: This simulation is different from the trajectory optimizer because the servo motors are assumed to have only the position interface.
This is much closer to the real mini-pupper robot than the trajectory optimizer in which the torque interface is assumed.

<img src="https://raw.githubusercontent.com/wiki/mayataka/mini_pupper_trajopt/images/mini_pupper_gazebo.gif" width="600"> 

## Summary for the Gazebo simulation
1. Install `Pinocchio`, `robotoc`, and `meshcat-python`.
2. Run `python3 trotting.py` at `mini_pupper_trajopt` directory.
3. Run `catkin_make` at ROS workspace. This command also installs generated trajectory files, e.g., `mini_pupper_trajopt/rsc/trotting/q.log`.
4. Launch Gazebo simulation as `roslaunch mini_pupper_gazebo mini_pupper.launch`