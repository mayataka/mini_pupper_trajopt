import mini_pupper_trajopt 
import robotoc


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('bounding')
trajopt.solve()
trajopt.visualize()

# logger = robotoc.utils.Logger(vars=['q', 'v'], root_dir='bounding')
# logger.take_log(trajopt.ocp_solver)