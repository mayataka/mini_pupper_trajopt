import mini_pupper_trajopt 


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('pacing')
trajopt.solve(num_iteration=150)
trajopt.visualize()

qs = trajopt.ocp_solver.get_solution('q')
vs = trajopt.ocp_solver.get_solution('v')
us = trajopt.ocp_solver.get_solution('u')
mini_pupper_trajopt.logger.take_log(vars=qs, file_name='q', root_dir='rsc/pacing')
mini_pupper_trajopt.logger.take_log(vars=vs, file_name='v', root_dir='rsc/pacing')
mini_pupper_trajopt.logger.take_log(vars=us, file_name='u', root_dir='rsc/pacing')

stages0 = trajopt.ocp_solver_factory.get_one_cycle_stage_indices(cycle=0)
q_initial_cycle = [qs[i] for i in stages0] 
v_initial_cycle = [vs[i] for i in stages0] 
mini_pupper_trajopt.logger.take_log(vars=q_initial_cycle, file_name='q_initial_cycle', root_dir='rsc/pacing')
mini_pupper_trajopt.logger.take_log(vars=v_initial_cycle, file_name='v_initial_cycle', root_dir='rsc/pacing')

stages = trajopt.ocp_solver_factory.get_one_cycle_stage_indices(cycle=1)
q_cycle = [qs[i] for i in stages] 
v_cycle = [vs[i] for i in stages] 
mini_pupper_trajopt.logger.take_log(vars=q_cycle, file_name='q_cycle', root_dir='rsc/pacing')
mini_pupper_trajopt.logger.take_log(vars=v_cycle, file_name='v_cycle', root_dir='rsc/pacing')