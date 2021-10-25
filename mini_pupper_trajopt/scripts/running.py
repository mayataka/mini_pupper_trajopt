import mini_pupper_trajopt 
import robotoc


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('running')
trajopt.solve(num_iteration=200)
trajopt.visualize(camera_tf_vec=[0.8, -2.0, -0.6], zoom=3.0)

logger = robotoc.utils.Logger(vars=['q', 'v'], root_dir='running')
logger.take_log(trajopt.ocp_solver)