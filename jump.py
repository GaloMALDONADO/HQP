import robot_config as conf
from wrapper import Wrapper
from viewer_utils import Viewer
from tasks import *
from simulator import Simulator
from pinocchio.utils import zero as mat_zeros
import trajectories as traj 
from solvers import NProjections

#__ Config
viewerName = 'Parkour'
robotName = "Skelette"
model_path = conf.model_path
#__ create the robot
robot = Wrapper(model_path)
dt = conf.dt
q0 = conf.half_sitting
v0 = mat_zeros(robot.nv)
#__ Create simulator: robot + viewer
simulator = Simulator('Sim1', q0, v0, 0.1, robotName, robot)
nq = simulator.robot.nq
nv = simulator.robot.nv
#simulator.viewer.updateRobotConfig(q0, robotName)

#__ Create Solver
solver =NProjections('Solv1', q0, v0, 0.1, robotName, robot)
#se3.computeJacobians(solver.robot.model, solver.robot.data, solver.)

#__ Create Tasks
# get desired state
idxRF = solver.robot.model.getFrameId('mtp_r')
oMf = solver.robot.framePosition(idxRF)
#p = conf.trajectories_path 
#pr_q1_ref = np.asmatrix(np.load(p+'prepare_ref1.npy')).T
oMf.transtaltion = oMf.translation*3
# create trjactory
RFtraj = traj.ConstantSE3Trajectory('reach',oMf)
RFtraj.setReference(oMf)
# create task
reach = SE3Task(solver.robot, idxRF, RFtraj)
reach.expDecay = True
#kp
#kv

#__ Stack the tasks
solver.addTask(reach, 1)
for i in range(100):
    qdot = solver.inverseKinematics1st(0)
    simulator.increment(robot.q, qdot)


#solver.tasks[0].kin_value

#def __init
