import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('running')
trajopt.solve(num_iteration=200)
trajopt.visualize(camera_tf_vec=[0.3, -1.5, -0.4], zoom=3.0)