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
        #self.nq = self.robot.nq
        #self.nv = self.robot.nv
        #self.na = self.nv-6        

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
    
        
