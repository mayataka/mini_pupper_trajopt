import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('trotting_slow')
trajopt.solve()
trajopt.visualize()