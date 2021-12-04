import mini_pupper_trajopt 


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('jumping')
trajopt.solve()
trajopt.visualize()

qs = trajopt.ocp_solver.get_solution('q')
mini_pupper_trajopt.logger.take_log(vars=qs, file_name='q', root_dir='rsc/jumping')