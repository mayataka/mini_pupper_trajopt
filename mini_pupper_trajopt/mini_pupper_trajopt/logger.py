import os
import numpy as np


def take_log(vars, file_name, root_dir='./', precision='%.18e', delimiter=', '):
    log_dir = os.path.abspath(root_dir) 
    log = os.path.join(log_dir, file_name+'.log') 
    os.makedirs(log_dir, exist_ok=True)
    with open(log, mode='w') as f:
        f.write('')
        f.close()
    with open(log, 'a') as logf:
        np.savetxt(logf, vars, fmt=precision, delimiter=delimiter)