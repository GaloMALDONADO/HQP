import pinocchio as se3
import numpy as np
from pinocchio import SE3

#_ Reference behaviours
def exponentialDecay(kp):
    ''' 
    Returns a kv value given a kp value
    This control law imposes an exponential decay on the task
    i.e. when tracking a moving target
    '''
    kv = 2*np.sqrt(kp)
    return kv

def adaptativeGain(p_error, kmin, kmax, beta):
    '''
    Returns a kp value given:
    kmin is the minimum gain when the error approaches zero
    kmax is the maximum gain when the task is far
    beta regulates the velocity of the transition of the behaviour
    i.e. to reach a fixed target
    '''
    kp = ( (kmin - kmax)*np.exp(-beta*np.linalg.norm(p_error)) ) + kmax
    return kp

def errorInSE3(M, Mdes):        
    '''                                                                                                     
    Compute a 6-dim error vector (6x1 np.maptrix) caracterizing the difference                          
    between M and Mdes, both element of SE3.    
    '''
    error = se3.log(Mdes.inverse()*M)
    return error


class Task:
    def __init__(self, robot, name = "Task"):
        self.robot = robot
        self.name = name
        self.kp = 1
        self.kv = 1
        # reference behaviours
        self.expDecay = False
        self.adaptGain = False
        self.kmin = 1
        self.kmax = 10
        self.beta = 5

class SE3Task(Task):
    def __init__(self, robot, frame_id, ref_trajectory, name = "Task"):
        Task.__init__ (self, robot, name)
        self._frame_id = frame_id
        self._ref_trajectory = ref_trajectory
        # set default value to M_ref
        self._M_ref = SE3.Identity
        # mask over the desired euclidian axis
        self._mask = (np.ones(6)).astype(bool)
        # for local to global
        self._gMl = SE3.Identity()

    def mask(self, mask):
        assert len(mask) == 6, "The mask must have 6 elemets"
        self._mask = mask.astype(bool)

    @property
    def refConfiguration (self, M_ref):
        assert isinstance(M_ref, SE3), "M_ref is not an element of class SE3"
        self._M_ref = M_ref

    @property
    def refTrajectory(self):
        return self._ref_trajectory

    def jointPostition(self):
        return robot.position(robot.q, joint_id)
    
    def framePosition(self):
        return self.robot.framePosition(self._frame_id)

    def positionError(self, t):
        oMi = self.robot.framePosition(self._frame_id)
        M_ref, v_ref, a_ref = self._ref_trajectory(t)
        p_error = errorInSE3(oMi, M_ref)
        return p_error.vector[self._mask]
    
    def velocityError(self, t):
        oMi = self.robot.framePosition(self._frame_id);
        self._gMl.rotation = oMi.rotation
        v_frame = self.robot.frameVelocity(self._frame_id);
        M_ref, v_ref, a_ref  = self._ref_trajectory(t);
        v_error = v_frame - self._gMl.actInv(v_ref)
        return v_error.vector[self._mask];
    
    def kin_value(self, t, q, local_frame = True):
        oMi = self.robot.framePosition(self._frame_id)
        v_frame = self.robot.frameVelocity(self._frame_id)

        # Get the reference trajectory   
        M_des, v_ref, a_ref  = self._ref_trajectory(t)
        
        # Transformation from local to world    
        self._gMl.rotation = oMi.rotation 
        
        #_ Task functions:
        # Compute desired velocity
        p_error = errorInSE3(oMi, M_des)
        v_error = v_frame - self._gMl.actInv(v_ref)
        
        # porportional derivative task
        if self.expDecay is True:
            self.kv = exponentialDecay(self.kp)
        if self.adaptGain is True:
            self.kp = adaptativeGain(p_error.vector, self.kmin, self.kmax, self.beta)
        v_des = - self.kp * p_error.vector  - self.kv * v_error.vector
        J= self.robot.frameJacobian(q, self._frame_id, False)

        if(local_frame==False):
            v_des[:3] = self._gMl.rotation * v_des[:3];
            v_des[3:] = self._gMl.rotation * v_des[3:];
            J[:3,:] = self._gMl.rotation * J[:3,:];
            J[3:,:] = self._gMl.rotation * J[3:,:];
        return J[self._mask,:], v_des[self._mask]


    def dyn_value(self, t, q, v, local_frame = True):
        # Get the current configuration of the link
        oMi = self.robot.framePosition(self._frame_id);
        v_frame = self.robot.frameVelocity(self._frame_id)
        
        # Get the reference trajectory
        M_ref, v_ref, a_ref  = self._ref_trajectory(t)
        
        # Transformation from local to world    
        self._gMl.rotation = oMi.rotation 
        
        #_ Taks functions:
        # Compute error acceleration desired
        p_error= errorInSE3(oMi, M_ref);
        v_error = v_frame - self._gMl.actInv(v_ref)
        drift = self.robot.frameAcceleration(self._frame_id)
        drift.linear += np.cross(v_frame.angular.T, v_frame.linear.T).T    
        
        # porportional derivative task
        if self.expDecay is True:
            self.kv = self.exponentialDecay(self.kp)
        if self.adaptGain is True:
            self.kp = self.adaptativeGain(p_error.vector, self.kmin, self.kmax, self.beta)
        a_des = - self.kp * p_error.vector - self.kv * v_error.vector + self._gMl.actInv(a_ref).vector #sign +-
        J = self.robot.frameJacobian(q, self._frame_id, False)
    
        if(local_frame==False):
            drift = self._gMl.act(drift);
            a_des[:3] = self._gMl.rotation * a_des[:3];
            a_des[3:] = self._gMl.rotation * a_des[3:];
            J[:3,:] = self._gMl.rotation * J[:3,:];
            J[3:,:] = self._gMl.rotation * J[3:,:];
        
        return J[self._mask,:], drift.vector[self._mask], a_des[self._mask]


    def jacobian(self, q, update_geometry = False):
        self.__jacobian_value = self.robot.frameJacobian(q, self._frame_id, update_geometry)
        return self.__jacobian_value[self._mask,:] 
    

