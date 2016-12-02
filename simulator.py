from wrapper import Wrapper
from viewer_utils import Viewer
import numpy as np 
from pinocchio.utils import zero as mat_zeros

class Simulator(object):
    DISPLAYCOM = True
    DISPLAYJOINTFRAMES = False
    COM_SPHERE_RADIUS = 0.01
    COM_SPHERE_COLOR = (1, 0, 0, 1)

    def reset(self, t, q, v, dt):
        n = self.nv
        self.dt = dt
        self.q = np.matrix.copy(q)
        self.v = np.matrix.copy(v)
        self.vOld = np.matrix.copy(v)
        self.dv = np.asmatrix(np.zeros(n)).T

    def __init__(self, name, q, v, dt, robotName, model_path):
        self.name = name
        self.time_step = 0
        self.robot = Wrapper(model_path)
        self.nq = self.robot.nq
        self.nv = self.robot.nv
        self.na = self.nv-6
        self.viewer=Viewer(self.name, self.robot, robotName)
        self.reset(0, q, v, dt)
        self.viewer.updateRobotConfig(q, robotName)
        if(self.DISPLAYCOM):
            self.viewer.addSphere('com', self.COM_SPHERE_RADIUS, mat_zeros(3), mat_zeros(3), self.COM_SPHERE_COLOR, 'OFF')
        

    ''' ********** SET ROBOT STATE ******* '''
    def setPositions(self, q):
        self.q = np.matrix.copy(q);
        self.viewer.updateRobotConfig(q)
        return self.q
    
    def setVelocities(self, v):
        self.v = np.matrix.copy(v);
        return self.v;
    
    def increment(self, q, qdot, updateViewer=True):
        q_next = se3.integrate(self.model,q,dq)
        q[:] = q_next[:]
        self.q = copy(q)
        self.viewer.updateRobotConfig(self.q)
        
    def integrateAcc(self, t, dt, dv, f, tau, updateViewer=True):
        res = [];
        self.t = t;
        self.time_step += 1;
        self.dv = np.matrix.copy(dv);
        
        if(abs(norm(self.q[3:7])-1.0) > EPS):
            print "SIMULATOR ERROR Time %.3f "%t, "norm of quaternion is not 1=%f" % norm(self.q[3:7]);
            
        ''' Integrate velocity and acceleration '''
        self.q  = se3.integrate(self.r.model, self.q, dt*self.v);
        self.v += dt*self.dv;
    
    ''' ---------- VIEWER ------------ '''
    def updateComPositionInViewer(self, com):
        assert np.asarray(com).squeeze().shape[0]==3, "com should be a 3x1 matrix"
        com = np.asarray(com).squeeze();
        if(self.time_step%int(self.VIEWER_DT/self.dt)==0):
            if(self.DISPLAY_COM):
                self.viewer.updateObjectConfig('com', (com[0], com[1], com[2], 0.,0.,0.,1.));
    
        
