import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('pacing')
trajopt.solve()
trajopt.visualize()