import mini_pupper_trajopt 
import robotoc


trajopt = mini_pupper_trajopt.TrajectoryOptimizer('trotting_slow')
trajopt.solve()
trajopt.visualize()

logger = robotoc.utils.Logger(vars=['q', 'v'], root_dir='trotting_slow')
logger.take_log(trajopt.ocp_solver)