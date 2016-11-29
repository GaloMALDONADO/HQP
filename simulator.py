import wrapper
class Simulator:
    def __init__(self, robotName, model_path, q):
        self.time_step = 0
        self.robot = wrapper.Wrapper(model_path)
        self.viewer=Viewer(self.name, self.robot)
        

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
    
    
        
