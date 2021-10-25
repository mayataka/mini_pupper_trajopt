import mini_pupper_trajopt 
import robotoc


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('pacing')
trajopt.solve()
trajopt.visualize()

logger = robotoc.utils.Logger(vars=['q', 'v'], root_dir='pacing')
logger.take_log(trajopt.ocp_solver)