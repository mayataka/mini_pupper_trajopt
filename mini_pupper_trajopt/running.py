import mini_pupper_trajopt 


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('running')
trajopt.solve(num_iteration=200)
trajopt.visualize(camera_tf_vec=[0.8, -2.0, -0.6], zoom=3.0)

qs = trajopt.ocp_solver.get_solution('q')
mini_pupper_trajopt.logger.take_log(vars=qs, file_name='q', root_dir='rsc/running')

stages0 = trajopt.ocp_solver_factory.get_one_cycle_stage_indices(cycle=0)
q_initial_cycle = [qs[i] for i in stages0] 
mini_pupper_trajopt.logger.take_log(vars=q_initial_cycle, file_name='q_initial_cycle', root_dir='rsc/running')

stages = trajopt.ocp_solver_factory.get_one_cycle_stage_indices(cycle=1)
q_cycle = [qs[i] for i in stages] 
mini_pupper_trajopt.logger.take_log(vars=q_cycle, file_name='q_cycle', root_dir='rsc/running')