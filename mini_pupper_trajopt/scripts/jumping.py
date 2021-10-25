import mini_pupper_trajopt 
import robotoc


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('jumping')
trajopt.solve()
trajopt.visualize()

logger = robotoc.utils.Logger(vars=['q', 'v'], root_dir='jumping')
logger.take_log(trajopt.ocp_solver)