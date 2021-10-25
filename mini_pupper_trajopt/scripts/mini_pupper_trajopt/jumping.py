import robotoc
import numpy as np
import math
from . import config


class JumpingOCPSolverFactory:
    def __init__(self, path_to_urdf=config.PATH_TO_URDF):
        self.path_to_urdf = path_to_urdf 
        # # medium jump
        self.jump_length = 0.20
        self.jump_height = 0.17
        self.flying_up_time = 0.15
        # # small jump
        # self.jump_length = 0.10
        # self.jump_height = 0.05
        # self.flying_up_time = 0.10
        self.flying_down_time = self.flying_up_time
        self.flying_time = self.flying_up_time + self.flying_down_time
        self.ground_time = 0.50
        self.dt = 0.005
        self.t0 = 0.
        self.T = self.t0 + self.flying_time + 2*self.ground_time
        self.N = math.floor(self.T/self.dt) 
        self.cycle = 10
        self.nthreads = 4

    def create_ocp_solver(self):
        LF_foot_id = config.LF_foot_id
        LH_foot_id = config.LH_foot_id
        RF_foot_id = config.RF_foot_id
        RH_foot_id = config.RH_foot_id
        contact_frames = [LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id] 
        baumgarte_time_step = 0.04
        robot = robotoc.Robot(self.path_to_urdf, robotoc.BaseJointType.FloatingBase, 
                              contact_frames, baumgarte_time_step)

        # create the cost function
        cost = robotoc.CostFunction()
        q_standing = config.q_standing
        q_weight = np.array([0, 0, 0, 250000, 250000, 250000, 
                            0.0001, 0.0001, 0.0001, 
                            0.0001, 0.0001, 0.0001,
                            0.0001, 0.0001, 0.0001,
                            0.0001, 0.0001, 0.0001])
        v_weight = np.array([100, 100, 100, 100, 100, 100, 
                            1, 1, 1, 
                            1, 1, 1,
                            1, 1, 1,
                            1, 1, 1])
        u_weight = np.full(robot.dimu(), 1.0e-01)
        qi_weight = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 
                            100, 100, 100, 
                            100, 100, 100,
                            100, 100, 100,
                            100, 100, 100])
        vi_weight = np.full(robot.dimv(), 100)
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

        com_ref0_flying_up = (q0_3d_LF + q0_3d_LH + q0_3d_RF + q0_3d_RH) / 4
        com_ref0_flying_up[2] = robot.com()[2]
        v_com_ref_flying_up = np.array([(0.5*self.jump_length/self.flying_up_time), 0, (self.jump_height/self.flying_up_time)])
        com_ref_flying_up = robotoc.PeriodicCoMRef(com_ref0_flying_up, v_com_ref_flying_up, 
                                                   self.t0+self.ground_time, self.flying_up_time, 
                                                   self.flying_down_time+2*self.ground_time, False)
        com_cost_flying_up = robotoc.TimeVaryingCoMCost(robot, com_ref_flying_up)
        com_cost_flying_up.set_q_weight(np.full(3, 1.0e06))
        cost.push_back(com_cost_flying_up)

        com_ref0_landed = (q0_3d_LF + q0_3d_LH + q0_3d_RF + q0_3d_RH) / 4
        com_ref0_landed[0] += self.jump_length
        com_ref0_landed[2] = robot.com()[2]
        v_com_ref_landed = np.zeros(3)
        com_ref_landed = robotoc.PeriodicCoMRef(com_ref0_landed, v_com_ref_landed, 
                                                self.t0+self.ground_time+self.flying_time, 
                                                self.ground_time, 
                                                self.ground_time+self.flying_time, False)
        com_cost_landed = robotoc.TimeVaryingCoMCost(robot, com_ref_landed)
        com_cost_landed.set_q_weight(np.full(3, 1.0e06))
        cost.push_back(com_cost_landed)

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
        max_num_impulses = 1
        contact_sequence = robotoc.ContactSequence(robot, max_num_impulses)

        contact_points = [q0_3d_LF, q0_3d_LH, q0_3d_RF, q0_3d_RH]
        contact_status_standing = robot.create_contact_status()
        contact_status_standing.activate_contacts([0, 1, 2, 3])
        contact_status_standing.set_contact_points(contact_points)
        contact_sequence.init_contact_sequence(contact_status_standing)

        contact_status_flying = robot.create_contact_status()
        contact_sequence.push_back(contact_status_flying, 
                                   self.t0+self.ground_time)

        contact_points[0][0] += self.jump_length
        contact_points[1][0] += self.jump_length
        contact_points[2][0] += self.jump_length
        contact_points[3][0] += self.jump_length
        contact_status_standing.set_contact_points(contact_points)
        contact_sequence.push_back(contact_status_standing, 
                                   self.t0+self.ground_time+self.flying_time)

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
    factory = JumpingOCPSolverFactory()
    ocp_solver = factory.create_ocp_solver()
    q = config.q_standing
    v = np.zeros(18)
    t = 0.

    num_iteration = 100
    robotoc.utils.benchmark.convergence(ocp_solver, t, q, v, num_iteration)

    viewer = robotoc.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                            base_joint_type=robotoc.BaseJointType.FloatingBase,
                                            viewer_type='meshcat')
    viewer.set_camera_transform_meshcat(camera_tf_vec=[0.3, -2.5, -0.4], zoom=7.0)
    viewer.display(factory.dt, ocp_solver.get_solution('q'))