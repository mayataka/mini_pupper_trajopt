import mini_pupper_trajopt 

trajopt = mini_pupper_trajopt.TrajectoryOptimizer('running')
trajopt.solve(num_iteration=200)
trajopt.ocp_solver_factory.dt *= 0.5
trajopt.visualize(camera_tf_vec=[0.8, -2.0, -0.6], zoom=3.0)