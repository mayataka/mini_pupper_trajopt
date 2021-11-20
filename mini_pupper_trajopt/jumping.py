import mini_pupper_trajopt 


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('jumping')
trajopt.solve()
trajopt.visualize()

qs = trajopt.ocp_solver.get_solution('q')
vs = trajopt.ocp_solver.get_solution('v')
mini_pupper_trajopt.logger.take_log(vars=qs, file_name='q', root_dir='rsc/jumping')
mini_pupper_trajopt.logger.take_log(vars=vs, file_name='v', root_dir='rsc/jumping')