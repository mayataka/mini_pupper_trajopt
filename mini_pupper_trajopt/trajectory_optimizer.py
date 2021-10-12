from . import trotting, trotting_slow, pacing, bounding, jumping, config
import numpy as np
import idocp


class TrajectoryOptimizer:
    def __init__(self, gait_type):
        if gait_type == 'trotting':
            self.ocp_solver, self.time_step = trotting.create_trotting_ocp_solver()
        elif gait_type == 'trotting_slow':
            self.ocp_solver, self.time_step = trotting_slow.create_trotting_slow_ocp_solver()
        elif gait_type == 'pacing':
            self.ocp_solver, self.time_step = pacing.create_pacing_ocp_solver()
        elif gait_type == 'bounding':
            self.ocp_solver, self.time_step = bounding.create_bounding_ocp_solver()
        elif gait_type == 'jumping':
            self.ocp_solver, self.time_step = jumping.create_jumping_ocp_solver()
        self.gait_type = gait_type
        self.q = config.q_standing
        self.v = np.zeros(18)
        self.t = 0.

    def solve(self, num_iteration=100):
        idocp.utils.benchmark.convergence(self.ocp_solver, self.t, self.q, 
                                          self.v, num_iteration)

    def visualize(self):
        viewer = idocp.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                            base_joint_type=idocp.BaseJointType.FloatingBase,
                                            viewer_type='meshcat')
        viewer.set_camera_transform_meshcat(camera_tf_vec=[0.3, -2.5, -0.4], zoom=7.0)
        viewer.display(self.time_step, self.ocp_solver.get_solution('q'))

    def get_state_trajectory(self):
        return self.ocp_solver.get_solution('q'), self.ocp_solver.get_solution('v')

    def get_acceleration_trajectory(self):
        return self.ocp_solver.get_solution('a')

    def get_contact_force_trajectory(self):
        return self.ocp_solver.get_solution('f')

    def get_control_input_trajectory(self):
        return self.ocp_solver.get_solution('u')

    def get_time_step(self):
        return self.time_step
