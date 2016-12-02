import numpy as np
from wrapper import Wrapper
#import
class NProjections:
    #def __init__(self, name, q, v, dt, robotName, model_path):
    def __init__(self, name, q, v, dt, robotName, robot):
        self.name = name
        self.time_step = 0
        self.robot = robot 
        self.robot.q = q
        self.robot.v = v
        #self.robot = Wrapper(model_path)
        self.nq = self.robot.nq
        self.nv = self.robot.nv
        self.na = self.nv-6
        self.dt = dt 
        self.t = 0.0
        self.tasks = []
        self.task_weights = []
    

    def null(A, eps=1e-12):
        '''Compute a base of the null space of A.'''
        u, s, vh = np.linalg.svd(A)
        padding = max(0,np.shape(A)[1]-np.shape(s)[0])
        null_mask = np.concatenate(((s <= eps), np.ones((padding,),dtype=bool)),axis=0)
        null_space = scipy.compress(null_mask, vh, axis=0)
        return scipy.transpose(null_space)

    def addTask(self, task, weight):
        '''
        append a new task and weight to the stack
        '''
        self.tasks        += [task]
        self.task_weights += [weight]

    def removeTask(self, task_name):
        for (i,t) in enumerate(self.tasks):
            if t.name==task_name:
                del self.tasks[i]
                del self.task_weights[i]
                return True
            raise ValueError("[InvDynForm] ERROR: task %s cannot be removed because it does not exist!" % task_name);

    def inverseKinematics1st(self,t):
        ''' 
        Hierarchichal Inverse Kinematics formulation based on nullspace projection
        q_dot
        ERRstack contain a stack of desired velocities in the operational space
        '''
        Jstack = []
        Errstack = []
        if len(self.tasks)==1:
            J, E = self.tasks[0].kin_value(t, self.robot.q)
            print J
            print E
            q_dot = np.linalg.pinv(J)*E
            #Z = self.null(J)
            return q_dot

        else:
            for i in xrange (len(self.tasks)):
                task = np.vstack([self.tasks[i]])
                Jstack, ERRstack = self.tasks[i].kin_value(t, self.robot.q)
                q_dot = np.linalg.pinv(Jstack[0])*ERRstack[0]
                Z = self.null(Jstack[0])

        if len(Jstack) > 0:
            for k in range (1, len(Jstack)):
                Jplus = Z[k-1]*np.linalg.pinv(Jstack[k]*Z[k-1])
                x_dot = (ERRstack[k] - Jstack[k]*qdot[k-1])
                q_dot += Jplus*x_dot
                Z = self.null(Jplus) 
        return q_dot
    
    def inverseKinematics2nd(self, Jstack, ERRstack, drift):
        ''' 
        Hierarchichal Inverse Kinematics formulation based on nullspace projection
        for q_dot_dot
        ERRstack contain the desired accelerations in the operation space
        '''
        q_dot_dot = np.linalg.pinv(Jstack[0]) * ( ERRstack[0] - drift)
        Z = self.null(Jstack[0])
        if len(Jstack) > 0:
            for k in range (1, len(Jstack)):
                Jplus = Z[k-1]*np.linalg.pinv(Jstack[k]*Z[k-1])
                x_dot = (ERRstack[k] - Jstack[k]*qdot[k-1])
                q_dot += Jplus*x_dot
                Z = self.null(Jplus) 
        return q_dot

    def inverseDynamics(self, robot, Jstack, drift, ERRstack):
        ''' 
        Hierarchichal Inverse Dynamics formulation based on nullspace projection
        '''
        #coriolis and centrifugal terms + gravity vector
        b = 1
        tau += np.linalg.pinv(Jstack[0]/robot.M) * ( ERRstack+((J/robot.M)*b)-dift )
        Z += self.null(Jstack[k]/robot.M) * Z[k-1]
        return Z

class qpOASES:
    def __init__(self):
        pass
