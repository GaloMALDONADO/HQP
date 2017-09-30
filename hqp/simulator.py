#from wrapper import Wrapper
from viewer_utils import Viewer
import numpy as np 
from pinocchio.utils import zero as mat_zeros
import pinocchio as se3
EPS = 1e-4

class Simulator(object):
    DISPLAYCOM = True
    DISPLAYJOINTFRAMES = False
    COM_SPHERE_RADIUS = 0.01
    COM_SPHERE_COLOR = (1, 0, 0, 1)

    '''
    name = ''
    q = None;   # current positions
    v = None;   # current velocities

    LCP = None;   # approximate LCP solver using staggered projections
    
    USE_LCP_SOLVER          = True;
    DETECT_CONTACT_POINTS   = True;   
    #'' True: detect collisions between feet and ground, False: collision is specified by the user ''
    GROUND_HEIGHT           = 0.0;
    LOW_PASS_FILTER_INPUT_TORQUES = False;
    
    ENABLE_FORCE_LIMITS = True;
    ENABLE_TORQUE_LIMITS = True;
    ENABLE_JOINT_LIMITS = True;
    
    ACCOUNT_FOR_ROTOR_INERTIAS = False;

    VIEWER_DT = 0.05;
    DISPLAY_COM = True;
    DISPLAY_CAPTURE_POINT = True;
    COM_SPHERE_RADIUS           = 0.01;
    CAPTURE_POINT_SPHERE_RADIUS = 0.01;
    CONTACT_FORCE_ARROW_RADIUS  = 0.01;
    COM_SPHERE_COLOR            = (1, 0, 0, 1); # red, green, blue, alpha
    CAPTURE_POINT_SPHERE_COLOR  = (0, 1, 0, 1);    
    CONTACT_FORCE_ARROW_COLOR   = (1, 0, 0, 1);
    CONTACT_FORCE_ARROW_SCALE   = 1e-3;
    contact_force_arrow_names = [];  # list of names of contact force arrows
    
    SLIP_VEL_THR = 0.1;
    SLIP_ANGVEL_THR = 0.2;
    NORMAL_FORCE_THR = 5.0;
    JOINT_LIMITS_DQ_THR = 1e-1; #1.0;
    TORQUE_VIOLATION_THR = 1.0;
    DQ_MAX = 9.14286;
                       
    ENABLE_WALL_DRILL_CONTACT = False;
    wall_x = 0.5;
    wall_damping = np.array([30, 30, 30, 0.3, 0.3, 0.3]);
    
    k=0;    # number of contact constraints (i.e. size of contact force vector)
    na=0;   # number of actuated DoFs
    nq=0;   # number of position DoFs
    nv=0;   # number of velocity DoFs
    r=[];   # robot
    
    mu=[];          # friction coefficient (force, moment)
    fMin = 0;       # minimum normal force

    dt = 0;     # time step used to compute joint acceleration bounds
    qMin = [];  # joint lower bounds
    qMax = [];  # joint upper bounds
    
    #'' Mapping between y and tau: y = C*tau+c ''
    C = [];
    c = [];
    
    M = [];         # mass matrix
    Md = [];        # rotor inertia
    h = [];         # dynamic drift
    q = [];
    dq = [];

    x_com = [];     # com 3d position
    dx_com = [];    # com 3d velocity
    ddx_com = [];   # com 3d acceleration
    cp = None;      # capture point
#    H_lankle = [];  # left ankle homogeneous matrix
#    H_rankle = [];  # right ankle homogeneous matrix
    J_com = [];     # com Jacobian
    Jc = [];        # contact Jacobian
    
    Minv = [];      # inverse of the mass matrix
    Jc_Minv = [];   # Jc*Minv
    Lambda_c = [];  # task-space mass matrix (Jc*Minv*Jc^T)^-1
    Jc_T_pinv = []; # Lambda_c*Jc_Minv
    Nc_T = [];      # I - Jc^T*Jc_T_pinv
    S_T = [];       # selection matrix
    dJc_v = [];     # product of contact Jacobian time derivative and velocity vector: dJc*v
    
    candidateContactConstraints = [];
    rigidContactConstraints = [];
#    constr_rfoot = None;
#    constr_lfoot = None;
#    constr_rhand = None;
    
#    f_rh = [];      # force right hand
#    w_rf = [];      # wrench right foot
#    w_lf = [];      # wrench left foot
    
    #'' debug variables ''
    x_c = [];       # contact points
    dx_c = [];      # contact points velocities
    x_c_init = [];  # initial position of constrained bodies    
        
    viewer = None;
    '''
    def reset(self, t, q, v, dt, nv):
        n = nv
        self.robot.dt = 0.0025 #dt
        self.robot.t = 0. #t
        self.robot.time_step = 0
        self.robot.q = np.matrix.copy(q)
        self.robot.v = np.matrix.copy(v)
        self.robot.vOld = np.matrix.copy(v)
        self.robot.dv = np.asmatrix(np.zeros(n)).T
        self.robot.time_step = 0
    
    def __init__(self, name, robot=None):
        if robot is None:
            pass
        self.name = name
        self.robot = robot
        self.reset(0, robot.q, robot.v, robot.dt, robot.nv)
        self.viewer = Viewer(self.robot.name, self.robot)
        self.updateRobotConfig(self.robot.q0)
        self.viewer.display(self.robot.q0,robot.name)
        #self.fMin = 0.001 
        #self.mu = np.array([ 0.3,  0.1])
        #self.nq = self.robot.nq
        #self.nv = self.robot.nv
        #self.na = self.nv        

        #if(self.DISPLAYCOM):
        #    self.viewer.addSphere('com', self.COM_SPHERE_RADIUS, mat_zeros(3), mat_zeros(3), self.COM_SPHERE_COLOR, 'OFF')
    
    def updateRobotConfig(self, q): 
        se3.computeAllTerms(self.robot.model,
                            self.robot.data,
                            q,
                            self.robot.v)
        se3.framesKinematics(self.robot.model, self.robot.data, q)
        se3.computeJacobians(self.robot.model, self.robot.data, q)

    ''' ********** SET ROBOT STATE ******* '''
    def setPositions(self, q):
        self.q = np.matrix.copy(q);
        self.viewer.updateRobotConfig(q)
        return self.q
    
    def setVelocities(self, v):
        self.v = np.matrix.copy(v);
        return self.v;
    
    def increment(self, q, v, dt, t, updateViewer=True):
        self.t = t
        self.time_step +=1 
        self.robot.v = v.copy()*dt
        self.q = se3.integrate(self.robot.model, q.copy(), self.robot.v)
        self.viewer.updateRobotConfig(self.q, self.robotName )
        
    def increment2(self, q, dv, dt, t, updateViewer=True):
        self.robot.t = t
        self.robot.time_step +=1 
        self.robot.dv = dv.copy()
        if(abs(np.linalg.norm(self.robot.q[3:7])-1.0) > EPS):
            print "SIMULATOR ERROR Time %.3f "%t, 
            "norm of quaternion is not 1=%f" % np.linalg.norm(self.robot.q[3:7])
        self.robot.q  = se3.integrate(self.robot.model, q.copy(), self.robot.v*dt)
        self.robot.v += dv.copy()*dt 
        self.updateRobotConfig(self.robot.q)
        if updateViewer is True:
            self.viewer.display(self.robot.q.copy(), self.robot.name)
        
    def integrateAcc(self, t, dt, dv, f, tau, updateViewer=True):
        self.t = t;
        self.time_step += 1;
        self.dv = np.matrix.copy(dv);
        
        if(abs(norm(self.q[3:7])-1.0) > EPS):
            print "SIMULATOR ERROR Time %.3f "%t, "norm of quaternion is not 1=%f" % norm(self.q[3:7]);
            
        ''' Integrate velocity and acceleration '''
        self.q  = se3.integrate(self.robot.model, self.q, dt*self.v);
        self.v += dt*self.dv;
        self.viewer.updateRobotConfig(self.q, self.robotName ) 
    
    ''' ---------- VIEWER ------------ '''
    def updateComPositionInViewer(self, com):
        assert np.asarray(com).squeeze().shape[0]==3, "com should be a 3x1 matrix"
        com = np.asarray(com).squeeze();
        if(self.time_step%int(self.VIEWER_DT/self.dt)==0):
            if(self.DISPLAY_COM):
                self.viewer.updateObjectConfig('com', (com[0], com[1], com[2], 0.,0.,0.,1.));
    
        
