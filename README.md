# Trajectory optimization package for Mini-Pupper robot
Purpose of this repository is to provide low-torque and low-impact trajectory for Mini-Pupper quadrupedal robot.

# Requirements 
- [idocp](https://github.com/mayataka/idocp.git)
First, install Pinocchio. Then install idocp at IDOCP_INSTALL_DIR

```
git clone https://github.com/mayataka/idocp & cd idocp
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOPTIMIZE_FOR_NATIVE=ON -DCMAKE_INSTALL_PREFIX=IDOCP_INSTALL_DIR
make install -j4
```

and add Python path as



- [meshcat-python](https://github.com/rdeits/meshcat-python.git)
Install meshcat-python as 
```
pip3 install meshcat-python
```


