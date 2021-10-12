import numpy as np
import os


current_path = os.path.dirname(__file__)
PATH_TO_URDF = os.path.join(current_path, 'mini_pupper_description/urdf/mini_pupper_description.urdf')

# Frame indices of foots
LF_foot_id = 18
LH_foot_id = 10
RF_foot_id = 34
RH_foot_id = 26

# Default configuration
q_standing = np.array([0, 0, 0.064, 0, 0, 0, 1, 
                       0.0, 0.8, -1.4,   # LH 
                       0.0, 0.8, -1.4,   # LF
                       0.0, 0.8, -1.4,   # RH
                       0.0, 0.8, -1.4])  # RF