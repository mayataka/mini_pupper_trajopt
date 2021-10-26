import mini_pupper_trajopt 


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('jumping')
trajopt.solve()
trajopt.visualize()

qs = trajopt.ocp_solver.get_solution('q')
vs = trajopt.ocp_solver.get_solution('v')