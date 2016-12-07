import numpy as np
from wrapper import Wrapper
import scipy
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
        self.sot = []

    def null(self, A, eps=1e-12):
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
        #self.sot.append(task.name)
        
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
        ERRstack = []
        Z = []
        if len(self.tasks)==1:
            J, E = self.tasks[0].kin_value(t, self.robot.q)
            q_dot = np.linalg.pinv(J)*E
            return q_dot

        else:
            for i in xrange (len(self.tasks)):
                #_Stack jacobians and task functions

                # if tasks are combined at same hierarchy
                if np.size(self.tasks[i]) > 1:
                    j = []
                    e = []
                    for w in xrange(len(self.tasks[i])):
                        x1, x2 = self.tasks[i][w].kin_value(t, self.robot.q)
                        j.append(x1)
                        e.append(x2)
                        J = np.vstack(j)
                        E = np.vstack(e)
                        
                else :        
                    J, E = self.tasks[i].kin_value(t, self.robot.q)
                    
                # stack them
                Jstack.append(J)
                ERRstack.append(E)

            #_Solve HQP
            q_dot = np.linalg.pinv(Jstack[0])*ERRstack[0]
            Z = self.null(Jstack[0]) 
            for k in range (1, len(Jstack)):
                Jplus = Z*np.linalg.pinv(Jstack[k]*Z) 
                x_dot = (ERRstack[k] - Jstack[k]*q_dot) 
                q_dot += Jplus*x_dot
                Z = self.null(Jstack[k]*Z*Z.T) 

        return q_dot
    
    def inverseKinematics2nd(self, t):
        ''' 
        Hierarchichal Inverse Kinematics formulation based on nullspace projection
        for q_dot_dot
        ERRstack contain the desired accelerations in the operation space
        '''
        Jstack = []
        ERRstack = []
        Dstack = []
        Z = []
        if len(self.tasks)==1:
            J, D, E = self.tasks[0].dyn_value(t, self.robot.q, self.robot.v)
            q_dot_dot = np.linalg.pinv(J)*(E - D)
            return q_dot_dot

        else:
            for i in xrange (len(self.tasks)):
                #_Stack jacobians and task functions

                # if tasks are combined at same hierarchy
                if np.size(self.tasks[i]) > 1:
                    j = []; e = []; d =[]
                    for w in xrange(len(self.tasks[i])):
                        x1, x2, x3 = self.tasks[i][w].dyn_value(t, self.robot.q, self.robot.v)
                        j.append(x1); d.append(x2); e.append(x3);
                        J = np.vstack(j); D = np.vstack(d); E = np.vstack(e);
                        
                else :        
                    J,D, E = self.tasks[i].dyn_value(t, self.robot.q, self.robot.v)
                # stack them
                Jstack.append(J); Dstack.append(D); ERRstack.append(E);

            #_Solve HQP
            q_dot_dot = np.linalg.pinv(Jstack[0])*(ERRstack[0]-Dstack[0])
            Z = self.null(Jstack[0]) 
            for k in range (1, len(Jstack)):
                Jplus = Z*np.linalg.pinv(Jstack[k]*Z) 
                x_dot_dot = (ERRstack[k] - Jstack[k]*q_dot_dot) 
                q_dot_dot += Jplus*(x_dot_dot-Dstack[k])
                Z = self.null(Jstack[k]*Z*Z.T) 

        return q_dot_dot

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
