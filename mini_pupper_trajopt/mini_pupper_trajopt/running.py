import robotoc
import numpy as np
import math
from . import config


class RunningOCPSolverFactory:
    def __init__(self, path_to_urdf=config.PATH_TO_URDF):
        self.path_to_urdf = path_to_urdf 
        self.step_length = 0.20
        self.step_height = 0.05
        self.front_swing_time = 0.12
        self.hip_swing_time   = 0.13
        self.flying_time      = 0.07
        self.running_time     = self.front_swing_time + self.hip_swing_time + self.flying_time
        self.dt = 0.005
        self.t0 = 0.5
        self.cycle = 10
        self.T = self.t0 + self.cycle*self.running_time + 2*self.dt
        self.N = math.floor(self.T/self.dt) 
        self.nthreads = 4

    def get_one_cycle_stage_indices(self, cycle):
        period = self.running_time
        N_period = math.floor(period/self.dt)
        N0 = math.floor(self.t0/self.dt)
        return range(N0+N_period*cycle-1, N0+N_period*(cycle+1))

    def create_ocp_solver(self):
        LF_foot_id = config.LF_foot_id
        LH_foot_id = config.LH_foot_id
        RF_foot_id = config.RF_foot_id
        RH_foot_id = config.RH_foot_id
        contact_frames = [LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id] 
        baumgarte_time_step = 0.02
        robot = robotoc.Robot(self.path_to_urdf, robotoc.BaseJointType.FloatingBase, 
                              contact_frames, baumgarte_time_step)

        # create the cost function
        cost = robotoc.CostFunction()
        q_standing = config.q_standing
        q_weight = np.array([0, 0, 0, 1.0, 1.0, 1.0, 
                            0.0001, 0.0001, 0.0001, 
                            0.0001, 0.0001, 0.0001,
                            0.0001, 0.0001, 0.0001,
                            0.0001, 0.0001, 0.0001])
        v_weight = np.array([1, 1, 1, 100, 100, 100, 
                            1, 1, 1, 
                            1, 1, 1,
                            1, 1, 1,
                            1, 1, 1])
        u_weight = np.full(robot.dimu(), 1.0e-03)
        qi_weight = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 
                            100, 100, 100, 
                            100, 100, 100,
                            100, 100, 100,
                            100, 100, 100])
        vi_weight = v_weight
        dvi_weight = np.full(robot.dimv(), 1.0e-03) # penalty on the impulse change in the velocity

        config_cost = robotoc.ConfigurationSpaceCost(robot)
        config_cost.set_q_ref(q_standing)
        config_cost.set_q_weight(q_weight)
        config_cost.set_qf_weight(q_weight)
        config_cost.set_qi_weight(qi_weight)
        config_cost.set_v_weight(v_weight)
        config_cost.set_vf_weight(v_weight)
        config_cost.set_vi_weight(vi_weight)
        config_cost.set_dvi_weight(dvi_weight)
        config_cost.set_u_weight(u_weight)
        cost.push_back(config_cost)

        robot.forward_kinematics(q_standing)
        q0_3d_LF = robot.frame_position(LF_foot_id)
        q0_3d_LH = robot.frame_position(LH_foot_id)
        q0_3d_RF = robot.frame_position(RF_foot_id)
        q0_3d_RH = robot.frame_position(RH_foot_id)
        LF_t0 = self.t0 
        LH_t0 = self.t0 + self.front_swing_time + self.flying_time
        RF_t0 = self.t0 
        RH_t0 = self.t0 + self.front_swing_time + self.flying_time
        LF_foot_ref = robotoc.PeriodicFootTrackRef(q0_3d_LF, self.step_length, self.step_height, 
                                                   LF_t0, self.front_swing_time+self.flying_time, 
                                                   self.hip_swing_time, True)
        LH_foot_ref = robotoc.PeriodicFootTrackRef(q0_3d_LH, self.step_length, self.step_height, 
                                                   LH_t0, self.hip_swing_time+self.flying_time, 
                                                   self.front_swing_time, False)
        RF_foot_ref = robotoc.PeriodicFootTrackRef(q0_3d_RF, self.step_length, self.step_height, 
                                                   RF_t0, self.front_swing_time+self.flying_time, 
                                                   self.hip_swing_time, True)
        RH_foot_ref = robotoc.PeriodicFootTrackRef(q0_3d_RH, self.step_length, self.step_height, 
                                                   RH_t0, self.hip_swing_time+self.flying_time, 
                                                   self.front_swing_time, False)
        LF_cost = robotoc.TimeVaryingTaskSpace3DCost(robot, LF_foot_id, LF_foot_ref)
        LH_cost = robotoc.TimeVaryingTaskSpace3DCost(robot, LH_foot_id, LH_foot_ref)
        RF_cost = robotoc.TimeVaryingTaskSpace3DCost(robot, RF_foot_id, RF_foot_ref)
        RH_cost = robotoc.TimeVaryingTaskSpace3DCost(robot, RH_foot_id, RH_foot_ref)
        foot_track_weight = np.full(3, 1.0e06)
        LF_cost.set_q_weight(foot_track_weight)
        LH_cost.set_q_weight(foot_track_weight)
        RF_cost.set_q_weight(foot_track_weight)
        RH_cost.set_q_weight(foot_track_weight)
        cost.push_back(LF_cost)
        cost.push_back(LH_cost)
        cost.push_back(RF_cost)
        cost.push_back(RH_cost)

        com_ref0 = (q0_3d_LF + q0_3d_LH + q0_3d_RF + q0_3d_RH) / 4
        com_ref0[2] = robot.com()[2] 
        v_com_ref = np.zeros(3)
        v_com_ref[0] = self.step_length / self.running_time
        com_ref = robotoc.PeriodicCoMRef(com_ref0, v_com_ref, self.t0, 
                                         self.running_time, 0., True)
        com_cost = robotoc.TimeVaryingCoMCost(robot, com_ref)
        com_q_weight = np.full(3, 1.0e05)
        com_q_weight[2] = 0.
        com_cost.set_q_weight(com_q_weight)
        cost.push_back(com_cost)

        # create the constraints
        constraints           = robotoc.Constraints()
        joint_position_lower  = robotoc.JointPositionLowerLimit(robot)
        joint_position_upper  = robotoc.JointPositionUpperLimit(robot)
        # joint_velocity_lower  = robotoc.JointVelocityLowerLimit(robot)
        # joint_velocity_upper  = robotoc.JointVelocityUpperLimit(robot)
        # joint_torques_lower   = robotoc.JointTorquesLowerLimit(robot)
        # joint_torques_upper   = robotoc.JointTorquesUpperLimit(robot)
        mu = 1.5
        friction_cone         = robotoc.FrictionCone(robot, mu)
        constraints.push_back(joint_position_lower)
        constraints.push_back(joint_position_upper)
        # constraints.push_back(joint_velocity_lower)
        # constraints.push_back(joint_velocity_upper)
        # constraints.push_back(joint_torques_lower)
        # constraints.push_back(joint_torques_upper)
        constraints.push_back(friction_cone)
        constraints.set_barrier(1.0e-01)

        # create the contact sequence
        max_num_impulses = 2*self.cycle
        contact_sequence = robotoc.ContactSequence(robot, max_num_impulses)

        contact_points = [q0_3d_LF, q0_3d_LH, q0_3d_RF, q0_3d_RH]
        contact_status_standing = robot.create_contact_status()
        contact_status_standing.activate_contacts([0, 1, 2, 3])
        contact_status_standing.set_contact_points(contact_points)
        contact_sequence.init_contact_sequence(contact_status_standing)

        contact_status_front_swing = robot.create_contact_status()
        contact_status_front_swing.activate_contacts([1, 3])
        contact_status_front_swing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_front_swing, self.t0)

        contact_points[0][0] += 0.5 * self.step_length
        contact_points[2][0] += 0.5 * self.step_length
        contact_status_standing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_standing, 
                                   self.t0+self.front_swing_time)

        contact_status_hip_swing = robot.create_contact_status()
        contact_status_hip_swing.activate_contacts([0, 2])
        contact_status_hip_swing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_hip_swing, 
                                   self.t0+self.front_swing_time+self.flying_time)

        contact_points[1][0] += self.step_length
        contact_points[3][0] += self.step_length
        contact_status_flying = robot.create_contact_status()

        for i in range(self.cycle-2):
            t1 = self.t0 + (i+1)*self.running_time
            contact_status_front_swing.set_contact_points(contact_points)
            contact_sequence.push_back(contact_status_front_swing, t1)

            contact_status_flying.set_contact_points(contact_points)
            contact_sequence.push_back(contact_status_flying, 
                                                t1+self.front_swing_time)

            contact_points[0][0] += self.step_length
            contact_points[2][0] += self.step_length
            contact_points[1][0] += self.step_length
            contact_points[3][0] += self.step_length
            contact_status_hip_swing.set_contact_points(contact_points)
            contact_sequence.push_back(contact_status_hip_swing, 
                                       t1+self.front_swing_time+self.flying_time)

        t1 = self.t0 + (self.cycle-1)*self.running_time
        contact_status_front_swing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_front_swing, t1)

        contact_status_flying.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_flying, t1+self.front_swing_time)

        contact_points[0][0] += self.step_length
        contact_points[2][0] += self.step_length
        contact_points[1][0] += 0.5 * self.step_length
        contact_points[3][0] += 0.5 * self.step_length
        contact_status_hip_swing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_hip_swing, 
                                   t1+self.front_swing_time+self.flying_time)

        t1 = self.t0 + self.cycle*self.running_time
        contact_status_standing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_standing, t1)

        ocp_solver = robotoc.OCPSolver(robot, contact_sequence, cost, constraints, 
                                       self.T, self.N, nthreads=self.nthreads)

        t = 0.0
        q = q_standing
        v = np.zeros(robot.dimv())

        ocp_solver.set_solution("q", q)
        ocp_solver.set_solution("v", v)
        f_init = np.array([0.0, 0.0, 0.25*robot.total_weight()])
        ocp_solver.set_solution("f", f_init)

        ocp_solver.init_constraints(t)

        return ocp_solver



if __name__ == '__main__':
    factory = RunningOCPSolverFactory()
    ocp_solver = factory.create_ocp_solver()
    q = config.q_standing
    v = np.zeros(18)
    t = 0.

    num_iteration = 200
    robotoc.utils.benchmark.convergence(ocp_solver, t, q, v, num_iteration)

    viewer = robotoc.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                            base_joint_type=robotoc.BaseJointType.FloatingBase,
                                            viewer_type='meshcat')
    viewer.set_camera_transform_meshcat(camera_tf_vec=[0.3, -1.5, -0.4], zoom=3.0)
    viewer.display(factory.dt, ocp_solver.get_solution('q'))
