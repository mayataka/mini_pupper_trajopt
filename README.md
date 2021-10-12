# Trajectory optimization package for Mini-Pupper robot
Purpose of this repository is to provide low-torque and low-impact trajectory for Mini-Pupper quadrupedal robot.

# Requirements 
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 
- [idocp](https://github.com/mayataka/idocp.git)

First, install [Pinocchio](https://github.com/stack-of-tasks/pinocchio) by following the [instruction](https://stack-of-tasks.github.io/pinocchio/download.html). 
Next, install [idocp](https://github.com/mayataka/idocp.git) at IDOCP_INSTALL_DIR as

```
git clone https://github.com/mayataka/idocp & cd idocp
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPTIMIZE_FOR_NATIVE=ON -DCMAKE_INSTALL_PREFIX=IDOCP_INSTALL_DIR
make install -j4
```

and modify Python path as

```
export PYTHONPATH=IDOCP_INSTALL_DIR/lib/python3.8/site-packages:$PYTHONPATH 
```

e.g., write it in `~/.bashrc`.
