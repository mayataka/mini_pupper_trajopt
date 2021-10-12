import idocp
import numpy as np
import math
from . import config



def create_jumping_ocp_solver(path_to_urdf=config.PATH_TO_URDF):
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

    # # small jump
    # jump_length = 0.10
    # jump_height = 0.05
    # period_flying_up = 0.10

    # # medium jump
    jump_length = 0.20
    jump_height = 0.17
    period_flying_up = 0.15

    period_flying_down = period_flying_up
    period_flying = period_flying_up + period_flying_down
    period_ground = 0.50
    t0 = 0

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
    dvi_weight = np.full(robot.dimv(), 1.0e-03) # penalty on the impulse change in the velocity

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

    com_ref0_flying_up = (q0_3d_LF + q0_3d_LH + q0_3d_RF + q0_3d_RH) / 4
    com_ref0_flying_up[2] = robot.com()[2]
    v_com_ref_flying_up = np.array([(0.5*jump_length/period_flying_up), 0, (jump_height/period_flying_up)])
    com_ref_flying_up = idocp.PeriodicCoMRef(com_ref0_flying_up, v_com_ref_flying_up, 
                                            t0+period_ground, period_flying_up, 
                                            period_flying_down+2*period_ground, False)
    com_cost_flying_up = idocp.TimeVaryingCoMCost(robot, com_ref_flying_up)
    com_cost_flying_up.set_q_weight(np.full(3, 1.0e06))
    cost.push_back(com_cost_flying_up)

    com_ref0_landed = (q0_3d_LF + q0_3d_LH + q0_3d_RF + q0_3d_RH) / 4
    com_ref0_landed[0] += jump_length
    com_ref0_landed[2] = robot.com()[2]
    v_com_ref_landed = np.zeros(3)
    com_ref_landed = idocp.PeriodicCoMRef(com_ref0_landed, v_com_ref_landed, 
                                        t0+period_ground+period_flying, period_ground, 
                                        period_ground+period_flying, False)
    com_cost_landed = idocp.TimeVaryingCoMCost(robot, com_ref_landed)
    com_cost_landed.set_q_weight(np.full(3, 1.0e06))
    cost.push_back(com_cost_landed)

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

    T = t0 + period_flying + 2*period_ground
    N = math.floor(T/dt) 
    max_num_impulse_phase = 1

    nthreads = 4
    t = 0
    ocp_solver = idocp.OCPSolver(robot, cost, constraints, T, N, 
                                max_num_impulse_phase, nthreads)

    contact_points = [q0_3d_LF, q0_3d_LH, q0_3d_RF, q0_3d_RH]
    contact_status_initial = robot.create_contact_status()
    contact_status_initial.activate_contacts([0, 1, 2, 3])
    contact_status_initial.set_contact_points(contact_points)
    ocp_solver.set_contact_status_uniformly(contact_status_initial)

    contact_status_flying = robot.create_contact_status()
    ocp_solver.push_back_contact_status(contact_status_flying, t0+period_ground)

    contact_points[0][0] += jump_length
    contact_points[1][0] += jump_length
    contact_points[2][0] += jump_length
    contact_points[3][0] += jump_length
    contact_status_initial.set_contact_points(contact_points)
    ocp_solver.push_back_contact_status(contact_status_initial, t0+period_ground+period_flying)

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
    ocp_solver, time_step = create_jumping_ocp_solver()

    num_iteration = 100
    idocp.utils.benchmark.convergence(ocp_solver, t, q, v, num_iteration)

    viewer = idocp.utils.TrajectoryViewer(path_to_urdf=config.PATH_TO_URDF, 
                                          base_joint_type=idocp.BaseJointType.FloatingBase,
                                          viewer_type='meshcat')
    viewer.set_camera_transform_meshcat(camera_tf_vec=[0.3, -2.5, -0.4], zoom=7.0)
    viewer.display(time_step, ocp_solver.get_solution('q'))