import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('bounding')
trajopt.solve()
trajopt.visualize()