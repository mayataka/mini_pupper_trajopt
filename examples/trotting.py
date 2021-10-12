import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('trotting')
trajopt.solve()
trajopt.visualize()