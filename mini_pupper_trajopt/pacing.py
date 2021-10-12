import idocp
import numpy as np
import math
from . import config



def create_pacing_ocp_solver(path_to_urdf=config.PATH_TO_URDF):
    LF_foot_id = config.LF_foot_id
    LH_foot_id = config.LH_foot_id
    RF_foot_id = config.RF_foot_id
    RH_foot_id = config.RH_foot_id
    contact_frames = [LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id] 
    path_to_urdf = '../mini_pupper_description/urdf/mini_pupper_description.urdf'
    baumgarte_time_step = 0.04
    robot = idocp.Robot(path_to_urdf, idocp.BaseJointType.FloatingBase, 
                        contact_frames, baumgarte_time_step)

    dt = 0.005
    step_length = 0.04
    step_height = 0.02
    # step_height = 0.012
    period_swing = 0.1
    period_double_support = 0.01
    t0 = period_double_support 
    cycle = 10

    cost = idocp.CostFunction()
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
    dvi_weight = np.full(robot.dimv(), 1.0e-01) # penalty on the impulse change in the velocity

    config_cost = idocp.ConfigurationSpaceCost(robot)
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
    LF_t0 = t0 + period_swing + period_double_support
    LH_t0 = t0 + period_swing + period_double_support
    RF_t0 = t0 
    RH_t0 = t0 
    LF_foot_ref = idocp.PeriodicFootTrackRef(q0_3d_LF, step_length, step_height, 
                                            LF_t0, period_swing, 
                                            period_swing+2*period_double_support, False)
    LH_foot_ref = idocp.PeriodicFootTrackRef(q0_3d_LH, step_length, step_height, 
                                            LH_t0, period_swing, 
                                            period_swing+2*period_double_support, False)
    RF_foot_ref = idocp.PeriodicFootTrackRef(q0_3d_RF, step_length, step_height, 
                                            RF_t0, period_swing, 
                                            period_swing+2*period_double_support, True)
    RH_foot_ref = idocp.PeriodicFootTrackRef(q0_3d_RH, step_length, step_height, 
                                            RH_t0, period_swing, 
                                            period_swing+2*period_double_support, True)
    LF_cost = idocp.TimeVaryingTaskSpace3DCost(robot, LF_foot_id, LF_foot_ref)
    LH_cost = idocp.TimeVaryingTaskSpace3DCost(robot, LH_foot_id, LH_foot_ref)
    RF_cost = idocp.TimeVaryingTaskSpace3DCost(robot, RF_foot_id, RF_foot_ref)
    RH_cost = idocp.TimeVaryingTaskSpace3DCost(robot, RH_foot_id, RH_foot_ref)
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
    v_com_ref[0] = 0.5 * step_length / period_swing
    com_ref = idocp.PeriodicCoMRef(com_ref0, v_com_ref, t0, period_swing, 
                                period_double_support, True)
    com_cost = idocp.TimeVaryingCoMCost(robot, com_ref)
    com_cost.set_q_weight(np.full(3, 1.0e06))
    cost.push_back(com_cost)

    constraints           = idocp.Constraints()
    joint_position_lower  = idocp.JointPositionLowerLimit(robot)
    joint_position_upper  = idocp.JointPositionUpperLimit(robot)
    # joint_velocity_lower  = idocp.JointVelocityLowerLimit(robot)
    # joint_velocity_upper  = idocp.JointVelocityUpperLimit(robot)
    # joint_torques_lower   = idocp.JointTorquesLowerLimit(robot)
    # joint_torques_upper   = idocp.JointTorquesUpperLimit(robot)
    mu = 1.5
    friction_cone         = idocp.FrictionCone(robot, mu)
    constraints.push_back(joint_position_lower)
    constraints.push_back(joint_position_upper)
    # constraints.push_back(joint_velocity_lower)
    # constraints.push_back(joint_velocity_upper)
    # constraints.push_back(joint_torques_lower)
    # constraints.push_back(joint_torques_upper)
    constraints.push_back(friction_cone)
    constraints.set_barrier(1.0e-01)

    T = t0 + cycle*(2*period_double_support+2*period_swing)
    N = math.floor(T/dt) 
    max_num_impulse_phase = 2*cycle

    nthreads = 4
    t = 0
    ocp_solver = idocp.OCPSolver(robot, cost, constraints, T, N, 
                                max_num_impulse_phase, nthreads)

    contact_points = [q0_3d_LF, q0_3d_LH, q0_3d_RF, q0_3d_RH]
    contact_status_initial = robot.create_contact_status()
    contact_status_initial.activate_contacts([0, 1, 2, 3])
    contact_status_initial.set_contact_points(contact_points)
    ocp_solver.set_contact_status_uniformly(contact_status_initial)

    contact_status_even = robot.create_contact_status()
    contact_status_even.activate_contacts([0, 1])
    contact_status_even.set_contact_points(contact_points)
    ocp_solver.push_back_contact_status(contact_status_even, t0)

    contact_points[2][0] += 0.5 * step_length
    contact_points[3][0] += 0.5 * step_length
    contact_status_initial.set_contact_points(contact_points)
    ocp_solver.push_back_contact_status(contact_status_initial, t0+period_swing)

    contact_status_odd = robot.create_contact_status()
    contact_status_odd.activate_contacts([2, 3])
    contact_status_odd.set_contact_points(contact_points)
    ocp_solver.push_back_contact_status(contact_status_odd, 
                                        t0+period_swing+period_double_support)

    contact_points[0][0] += step_length
    contact_points[1][0] += step_length
    contact_status_initial.set_contact_points(contact_points)
    ocp_solver.push_back_contact_status(contact_status_initial, 
                                        t0+2*period_swing+period_double_support)

    for i in range(cycle-1):
        t1 = t0 + (i+1)*(2*period_swing+2*period_double_support)
        contact_status_even.set_contact_points(contact_points)
        ocp_solver.push_back_contact_status(contact_status_even, t1)

        contact_points[2][0] += step_length
        contact_points[3][0] += step_length
        contact_status_initial.set_contact_points(contact_points)
        ocp_solver.push_back_contact_status(contact_status_initial, t1+period_swing)

        contact_status_odd.set_contact_points(contact_points)
        ocp_solver.push_back_contact_status(contact_status_odd, 
                                            t1+period_swing+period_double_support)

        contact_points[0][0] += step_length
        contact_points[1][0] += step_length
        contact_status_initial.set_contact_points(contact_points)
        ocp_solver.push_back_contact_status(contact_status_initial, 
                                            t1+2*period_swing+period_double_support)

    q = q_standing
    v = np.zeros(robot.dimv())

    ocp_solver.set_solution("q", q)
    ocp_solver.set_solution("v", v)
    f_init = np.array([0.0, 0.0, 0.25*robot.total_weight()])
    ocp_solver.set_solution("f", f_init)

    ocp_solver.init_constraints(t)

    return ocp_solver, dt



if __name__ == '__main__':
    q = config.q_standing
    v = np.zeros(18)
    t = 0.
    ocp_solver, time_step = create_pacing_ocp_solver()

    num_iteration = 100
    idocp.utils.benchmark.convergence(ocp_solver, t, q, v, num_iteration)

    viewer = idocp.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                          base_joint_type=idocp.BaseJointType.FloatingBase,
                                          viewer_type='meshcat')
    viewer.set_camera_transform_meshcat(camera_tf_vec=[0.3, -2.5, -0.4], zoom=7.0)
    viewer.display(time_step, ocp_solver.get_solution('q'))