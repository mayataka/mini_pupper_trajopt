import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('jumping')
trajopt.solve()
trajopt.visualize()