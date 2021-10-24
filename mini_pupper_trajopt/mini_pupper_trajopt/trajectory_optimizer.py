from . import trotting, pacing, bounding, jumping, running, config
import numpy as np
import robotoc


class TrajectoryOptimizer:
    def __init__(self, gait_type):
        if gait_type == 'trotting':
            self.ocp_solver_factory = trotting.TrottingOCPSolverFactory()
        elif gait_type == 'trotting_slow':
            self.ocp_solver_factory = trotting.TrottingOCPSolverFactory()
            self.ocp_solver_factory.swing_time = 0.2
            self.ocp_solver_factory.support_time = 0.02
            self.ocp_solver_factory.support_time = 0.01
        elif gait_type == 'pacing':
            self.ocp_solver_factory = pacing.PacingOCPSolverFactory()
        elif gait_type == 'bounding':
            self.ocp_solver_factory = bounding.BoundingOCPSolverFactory()
        elif gait_type == 'jumping':
            self.ocp_solver_factory = jumping.JumpingOCPSolverFactory()
        elif gait_type == 'running':
            self.ocp_solver_factory = running.RunningOCPSolverFactory()
        self.ocp_solver = self.ocp_solver_factory.create_ocp_solver()
        self.gait_type = gait_type
        self.q = config.q_standing
        self.v = np.zeros(18)
        self.t = 0.

    def solve(self, num_iteration=100):
        self.ocp_solver = self.ocp_solver_factory.create_ocp_solver()
        robotoc.utils.benchmark.convergence(self.ocp_solver, self.t, self.q, 
                                            self.v, num_iteration)

    def visualize(self, camera_tf_vec=[0.3, -2.5, -0.4], zoom=7.0):
        viewer = robotoc.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                                base_joint_type=robotoc.BaseJointType.FloatingBase,
                                                viewer_type='meshcat')
        viewer.set_camera_transform_meshcat(camera_tf_vec=camera_tf_vec, zoom=zoom)
        viewer.display(self.ocp_solver_factory.dt, self.ocp_solver.get_solution('q'))

    def get_state_trajectory(self):
        return self.ocp_solver.get_solution('q'), self.ocp_solver.get_solution('v')

    def get_acceleration_trajectory(self):
        return self.ocp_solver.get_solution('a')

    def get_contact_force_trajectory(self):
        return self.ocp_solver.get_solution('f')

    def get_control_input_trajectory(self):
        return self.ocp_solver.get_solution('u')

    def get_time_step(self):
        return self.ocp_solver_factory.dt
