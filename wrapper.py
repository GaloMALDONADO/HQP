# import buildTree, createHumanVisualsList and displayHuman
#from biomechanics.full_body_skeleton import *
import pinocchio as se3
from pinocchio.utils import zero
#from biomechanics.maths import *
import numpy as np
import time
import os
from Models import osim_parser
#from biomechanics.OpenSimParser import *
#from biomechanics.filters import *

class Wrapper():
    def __init__(self, model_path=None):
        if model_path is None:
            model_path = '/local/gmaldona/devel/biomechatronics/models/GX.osim'
        #self.osim = OpenSimTools()
        #PyModel = Osim.readOsim(model)
        #self.model, self.visuals = Osim.buildPinocchioModel(Osim.readOsim(model_path))
        r = osim_parser.Osim2PinocchioModel()
        r.parseModel(model_path)
        self.model = r.model 
        self.visuals = r.visuals
        self.data = r.data
        #self.osim.parseModel(model_path)
        #human = Skelete()
        #self.model = human.model
        #self.visuals = human.visuals
        #self.data = self.model.createData()
        self.v0 = zero(self.model.nv)
        self.q0 = zero(self.model.nq)
        self.q = self.q0
        self.oMp = se3.utils.rotate('z',np.pi/2) * se3.utils.rotate('x',np.pi/2)
        self.markersFreq = np.float(400.)
        self.dt = np.float(1/400.)
        #self.initDisplay()
        #self.half_sitting()
        #se3.centerOfMass(self.model, self.data, zero(self.model.nq), zero(self.model.nv),True)
        #se3.forwardKinematics(self.model, self.data, zero(self.model.nq), zero(self.model.nv))
        #se3.framesKinematics(self.model, self.data, zero(self.model.nq))

        #self.q0[6] = 1
        #self.visuals = createHumanVisualsList(self.model.names)
        #self.play(self.q0, self.v0)
        #self.lowerPoseLimit = {
        #    self.
        #self.maximumEfforts
        #    def updateBodyPlacements(self, q):
        #        se3.framesKinematics(self.model, self.data, q))
        #    def forwardKinematics(model, data, nq, nv):
        #        se3.forwardKinematics(model, data, nq, nv)
    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    def initDisplay(self, viewerRootNodeName="world/pinocchio", loadModel=False):
        import gepetto.corbaserver
        try:
            self.viewer=gepetto.corbaserver.Client()
            print "Connected to corba server"
            self.viewerRootNodeName = viewerRootNodeName
            if loadModel:
                self.loadDisplayModel(viewerRootNodeName)
        except:
            if 'viewer' in self.__dict__:
                del self.viewer
            print "!! Error while starting the viewer client. "
            print "Check whether Gepetto-viewer-corba is properly started"

    # Create the scene displaying the robot meshes in gepetto-viewer
    def loadDisplayModel(self, nodeName, windowName="pinocchio"):
        # Open a window for displaying your model.  
        try:
            # If the window already exists, do not do anything.                           
            self.windowID = self.viewer.gui.getWindowID (windowName)
            print "Warning: window '"+windowName+"' already created."
            print "The previously created objects will not be destroyed and do not have to be created again."
        except:
            # Otherwise, create the empty window.                                           
            self.windowID = self.viewer.gui.createWindow (windowName)
        # Start a new "scene" in this window, named "world", with just a floor.         
        if "world" not in self.viewer.gui.getSceneList():
          self.viewer.gui.createScene("world")
          #self.viewer.gui.createSceneWithFloor("world")
        self.viewer.gui.addSceneToWindow("world", self.windowID)
        
        self.viewer.gui.createGroup(nodeName)
            
        # iterate over visuals and create the meshes in the viewer 
        for i in range (1,len(self.visuals)):
            self.viewer.gui.addMesh('world/'+self.visuals[i][1]+os.path.split(self.visuals[i][2])[1], self.visuals[i][2])
        # iterate for creating nodes for all joint
        for i in range(1,self.model.nbodies):
            self.viewer.gui.addXYZaxis('world/'+self.model.names[i], [1., 0., 0., .5], 0.02, 1)
        
        # create a node for the center of mass
        self.viewer.gui.addXYZaxis('world/globalCoM', [0., 1., 0., .5], 0.03, 0.3)
        
        # Finally, refresh the layout to obtain your first rendering.           
        self.viewer.gui.refresh()


    def placeObject(self, objName, M, refresh=True):
        '''                                                                                    
        This function places (ie changes both translation and rotation) of the object names 
        "objName" in place given by the SE3 object "M". By default, immediately refresh the 
        layout. If multiple objects have to be placed at the same time, do the refresh only 
        at the end of the list 
        '''
        pinocchioConf = XYZQUATToViewerConfiguration(se3ToXYZQUAT(M)) 
        self.viewer.gui.applyConfiguration(objName,pinocchioConf)
        if refresh: self.viewer.gui.refresh()

    def addLandmark(self, nodeName, size):
        self.viewer.gui.addLandmark(nodeName, size)
    
    def displayJointLandmarks(self):
        #create nodes
        for i in range(1,self.model.nbodies):
            #idx = self.model.getJointId(self.visuals[i][1]) 
            pose =  self.data.oMi[i]
            #self.viewer.gui.addXYZaxis('world/'+self.model.names[i], [0., 1., 0., .5], 0.02, 0.1)
            #self.viewer.gui.addLandmark('world/'+self.model.names[i], 0.3)
            self.placeObject('world/'+self.model.names[i], pose, True)

    def getNodeList(self):
        return self.viewer.gui.getNodeList()

    def getSubTree(self, idx):
        subtree = []
        idx_p = self.model.parents[idx]
        #dof = human.model.joints[idx_p].idx_q
        #subtree.append([idx_p, dof])
        subtree.append(idx_p)
        while True:
            idx_p = self.model.parents[idx_p]
            #dof = human.model.joints[idx_p].idx_q
            #subtree.append([idx_p, dof])
            subtree.append(idx_p)
            if idx_p == 0:
                break
        return subtree

    def update(self,q):
        se3.forwardKinematics(self.model, self.data, q)
        se3.computeJacobians(self.model, self.data, q)
        self.q = q
        

    def display(self, q, osimref=True, com=True, updateKinematics=True):
        # update q kinematics 
        se3.forwardKinematics(self.model, self.data, q)
        se3.computeJacobians(self.model, self.data, q)
        #self.updateGeometryPlacements(q,visual=True)
        self.q = q.copy()

        # show CoM
        if com is True:
            CoM = se3.SE3.Identity()
            CoM.translation = self.com(q)
            self.placeObject("world/globalCoM", CoM, True)

        #display skelette visuals
        import os
        for i in range (1,len(self.visuals)):
            # get the parent joint
            idx = self.model.getJointId(self.visuals[i][1]) 
            pose = se3.SE3.Identity()
            if osimref is True:
                # convert bones pose
                pose.translation=  self.data.oMi[idx].translation + np.matrix(np.array([0.,0.,0.])*np.array(np.squeeze(self.visuals[i][4][3:6]))[0]).T
#*self.visuals[i][4][3:6])
                pose.rotation= self.data.oMi[idx].rotation * self.oMp
                
            else:
                pose.translation=  self.data.oMi[idx].translation+self.visuals[i][4][3:6]
                pose.rotation= self.data.oMi[idx].rotation

            # place objects
            if i != len(self.visuals)-1:
                self.viewer.gui.setScale("world/"+self.visuals[i][1]+os.path.split(self.visuals[i][2])[1], self.visuals[i][3]) 
                self.placeObject("world/"+self.visuals[i][1]+os.path.split(self.visuals[i][2])[1], pose, False)
            else:
                self.viewer.gui.setScale("world/"+self.visuals[i][1]+os.path.split(self.visuals[i][2])[1], self.visuals[i][3]) 
                self.placeObject("world/"+self.visuals[i][1]+os.path.split(self.visuals[i][2])[1], pose, True)
        

    def parseTrial(self, data):
        q=[]
        for i in range(0, len(data)):
            q.append( self.dof2pinocchio(data[i][:]) )
        return q
        
    def readOsim(self, filename):
        trial = self.osim.readData(filename)
        trial['osim_data'] = trial['data']
        trial['pinocchio_data'] = np.asmatrix(self.parseTrial(trial['data'][:]))
        trial['time'] = np.asmatrix(trial['time'][:])
        return trial
    
    def inverseDynamics(self, q, v, a, f_ext=None):
        '''ID(q, v, a, f_ext)
         f_ext: Vector of external forces expressed in the local frame of each joint 
         do it recursively
        '''
        if f_ext is None:
            for i in range(0, len(q)):
                Tau.append(se3.rnea(self.model, self.data, q, v, a))
        else:
            for i in range(0, len(q)):
                Tau.append(se3.rnea(self.model, self.data, q, v, a, f_ext))
        return Tau

    def forwardDynamics(self):
        pass

    def position(self, q, index, update_geometry=True):
        if update_geometry:
            se3.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[index] 

    def differentiate(self, q1, q2):
        return se3.differentiate(self.model, np.asmatrix(q1), np.asmatrix(q2))
        
    def generalizedVelocity(self, Q, dt):
        return np.asmatrix( np.gradient(Q, dt) )
        
    def generalizedAcceleration(self, V, dt):
        return np.asmatrix(np.gradient(V, dt))

    def velocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v)
        return self.data.v[index]

    def acceleration(self, q, v, a, index, update_acceleration=True):
        if update_acceleration:
            se3.forwardKinematics(self.model, self.data, q, v, a)
        return self.data.a[index]

    def forwardKinematics(self, q, v=None, a=None):
        if v is not None:
            if a is not None:
                se3.forwardKinematics(self.model, self.data, q, v, a)
            else:
                se3.forwardKinematics(self.model, self.data, q, v)
        else:
            se3.forwardKinematics(self.model, self.data, q)

    def computeAllKinematics(self, Q):
        self.Q = Q
        self.V = self.generalizedVelocity(self.Q, self.dt)
        self.A = self.generalizedAcceleration(self.V, self.dt)
        #se3.forwardKinematics(self.model, self.data, self.Q, self.V, self.A)
        
    def playForwardKinematics(self, Q, sleep=0.0025, step=10, record=False):
        ''' playForwardKinematics(q, sleep, step, record)
        '''
        # TODO at verical line to plot as in opensim during playing
        rec = {'q':[],'com':[], 'Jcom':[]}
        for i in range(0, len(Q),step):
            self.q = Q[i]
            self.display(self.q, osimref=True, com=True, updateKinematics=False)
            time.sleep(sleep)
            if record is True:
                #rec =  self.record()
                rec['q'].append(self.q)
                rec['com'].append(self.com(self.q).getA()[:,0])
                rec['Jcom'].append(self.Jcom(se3.jacobianCenterOfMass(self.model, self.data, self.q)))
        if record is True:
            return rec


    def record(self, motion, variable, idx=None):
        Jtask = []
        task = []
        if variable is 'joint':
            for i in range(0, len(motion)):
                se3.forwardKinematics(self.model, self.data, motion[i])
                #task['rotation'].append(se3.utils.matrixToRpy(self.data.oMi[idx].rotation))
                #task['translation'].append(self.data.oMi[idx].translation)
                #task.append([self.data.oMi[idx].translation, 
                #             se3.utils.matrixToRpy(self.data.oMi[idx].rotation)])
                #task.append(np.row_stack((self.data.oMi[idx].translation, 
                #                          se3.utils.matrixToRpy(self.data.oMi[idx].rotation))))
                task.append(np.row_stack((self.data.oMi[idx].translation, 
                                          np.matrix(euler_from_matrix(
                                              self.data.oMi[idx].rotation,'syxz')).T)))
                #M = (self.data.oMi[idx-1].rotation).T*self.data.oMi[idx].rotation
                #task.append(np.row_stack((self.data.oMi[idx].translation,
                #                          np.matrix(euler_from_matrix(M,'sxyz')).T)))
                #M = self.oMp*self.data.oMi[idx].rotation
                Jtask.append(se3.jacobian(self.model, self.data, motion[i], idx, True, True))

        elif variable is 'com':
            for i in range(0, len(motion)):
                se3.forwardKinematics(self.model, self.data, motion[i])
                task.append(self.com(motion[i]).getA()[:,idx])
                Jtask.append(self.Jcom(motion[i]))
        

        return task, Jtask
            
    def kine(self, motion):
        q = []
        vel = []
        #hg = []
        for i in range(0, len(motion)):
            se3.forwardKinematics(self.model, self.data, motion[i])
            q.append(motion[i])
            if i is 0:
                vel.append(self.v0)
            else:
                vel.append(self.differentiate(q[i-1], q[i])/self.dt)
        #filter
        v=self.reshape(vel)
        v_hat=self.filterM(v, cutoff=35, fs=400, order=4)
        v_hat2 =self.reshape(v_hat)
        #    se3.ccrba(self.model, self.data, q[i], vel[i])
        #hg.append(self.data.hg)
        return q, v_hat2
    
    def reshape(self, X):
        r,c = np.shape(X)[0:2]
        X_hat = np.zeros((r,c))
        for i in xrange(r):
            xi = np.array(X[i])
            for j in xrange(c):
                X_hat[i,j] = xi[j]
        return X_hat
        
    def filter(self, X, cutoff=10, fs=400, order=4):
        X_hat = filtfilt_butter(X, cutoff, fs, order)
        return X_hat


    def filterM(self, M, cutoff=10, fs=400, order=4):
        t, dof = np.shape(M)[0:2]
        M_hat = np.zeros((t,dof))
        for i in xrange(dof):
            M_hat[:,i] = filtfilt_butter(M[:,i], cutoff, fs, order)
        return M_hat

    def cam(self, q, vel):
        hg = []
        for i in range(0, len(q)):
            se3.ccrba(self.model, self.data, q[i], vel[i])
            hg.append(self.data.hg.copy())
        return hg

    def com(self, q, v=None, a=None, update_kinematics=True):
        if v is not None:
            if a is None:
                se3.centerOfMass(self.model, self.data, q, v, update_kinematics)
                return self.data.com[0], self.data.vcom[0]
            se3.centerOfMass(self.model, self.data, q, v, a, update_kinematics)
            return self.data.com[0], self.data.vcom[0], self.data.acom[0]
        return se3.centerOfMass(self.model, self.data, q, update_kinematics)
    
    def Jcom(self, q):
        return se3.jacobianCenterOfMass(self.model, self.data, q)
    
    def getDoF(self, jointName):
        idx = self.model.getJointId(jointName)
        if idx < len (self.model.joints):
            idx = self.model.joints[idx].idx_q
            return idx
        else:
            raise Exception('The body segment name is not recognized in skeletor model')
    
    def increment(self, q, dq):
        q_next = se3.integrate(self.model,q,dq)
        q[:] = q_next[:]

    def jacobian(self, q, index, update_geometry=True, local_frame=True):
        return se3.jacobian(self.model, self.data, q, index, local_frame, update_geometry)
    
    def computeJacobians(self, q):
        return se3.computeJacobians(self.model, self.data, q)

    def framePosition(self, index):
        f = self.model.frames[index]
        return self.data.oMi[f.parent].act(f.placement)

    def frameVelocity(self, index):
        f = self.model.frames[index]
        return f.placement.actInv(self.data.v[f.parent])
        
    ''' Return the spatial acceleration of the specified frame. '''
    def frameAcceleration(self, index):
        f = self.model.frames[index]
        return f.placement.actInv(self.data.a[f.parent])
        
    def frameClassicAcceleration(self, index):
        f = self.model.frames[index]
        a = f.placement.actInv(self.data.a[f.parent])
        v = f.placement.actInv(self.data.v[f.parent])
        a.linear += np.cross(v.angular.T, v.linear.T).T
        return a;
        ''' Call computeJacobians if update_geometry is true. 
        If not, user should call computeJacobians first. 
        Then call getJacobian and return the resulted jacobian matrix. 
        Attention: if update_geometry is true,the function computes 
        all the jacobians of the model. It is therefore outrageously 
        costly wrt a dedicated call. Use only with update_geometry for prototyping.
    '''
    def frameJacobian(self, q, index, update_geometry=True, local_frame=True):
        return se3.frameJacobian(self.model, self.data, q, index, local_frame, update_geometry)


    def dof2pinocchio(self, dof):
        ''' qPinocchio = dof2pinocchio(dof = GX generalize coordinates)
        
        - Convert OpenSim generalized coordinates vector into equivalent Pinocchio generalized coordinates
        - This function is designed to work with the GX model
        

        - GX OpenSim generalized coordinates:
        dof = [pelvis,  0..5
        rhip, rknee, rankle, rsubtalar, rmtp,  6..8, 9, 10, 11, 12
        lhip, lknee, lankle, lsubtalar, lmtp, 13..15, 16, 17, 18, 19
        back, neck, 20..22, 23..25
        rshoulder, relbow, rpro_sup, rwrist flexion, rwrist deviation, rfingers flexion 26..28, 29, 30, 31, 32, 33
        lshoulder, lelbow, lpro_sup, lwrist flexion, lwrist deviation, lfingers flexion 34..36, 37, 38, 39, 40, 41
        
        - Pinocchio correspondance:
        q = [pelvis,  0..6
        rhip, rknee, rankle, rsubtalar, rmtp,  7..10, 11, 12, 13, 14
        lhip, lknee, lankle, lsubtalar, lmtp, 15..18, 19, 20, 21, 22
        back, neck, 23..26, 27..30
        rshoulder, relbow, rpro_sup, rwrist flexion, rwrist deviation, rfingers flexion 31..34, 35, 36, 37, 38, 39
        lshoulder, lelbow, lpro_sup, lwrist flexion, lwrist deviation, lfingers flexion 40..43, 44, 45, 46, 47, 48
        '''
        #Change OpenSim values to correpond to Pinocchio model
        pt = np.squeeze(np.array( self.oMp * np.matrix(dof[3:6]).T )) #tx,ty,tz
        pelvis = quaternion_from_matrix(euler_matrix((dof[2]),dof[0],dof[1],'szxy'))
        #dof[39]=-dof[39]#wrist flexion l
        #dof[40]=-dof[40]#wrist deviation l
        rshoulder = quaternion_from_matrix(euler_matrix(dof[28],dof[26],dof[27],'rzxy'))
        lshoulder = quaternion_from_matrix(euler_matrix(-dof[36],dof[34],-dof[35],'rzxy'))
        rhip = quaternion_from_matrix(euler_matrix(dof[8],dof[6],dof[7],'szxy'))
        lhip = quaternion_from_matrix(euler_matrix(-dof[15],dof[13],-dof[14],'szxy'))
        back = quaternion_from_matrix(euler_matrix(dof[22],dof[20],dof[21],'szxy'))
        neck = quaternion_from_matrix(euler_matrix(dof[25],dof[23],dof[24],'szxy'))

        q = np.array([ tuple(pt[0:3])+tuple([pelvis[1],pelvis[2],pelvis[3],pelvis[0]])
                        +tuple([rhip[1],rhip[2],rhip[3],rhip[0]])+tuple(dof[9:13])
                        +tuple([lhip[1],lhip[2],lhip[3],lhip[0]])+tuple(dof[16:20])
                        +tuple([back[1],back[2],back[3],back[0]])
                        +tuple([neck[1],neck[2],neck[3],neck[0]])
                        +tuple([rshoulder[1],rshoulder[2],rshoulder[3],rshoulder[0]])+tuple(dof[29:34])
                        +tuple([lshoulder[1],lshoulder[2],lshoulder[3],lshoulder[0]])+tuple(dof[37:42])
                        ])[0]
        return q


    #test individual joints
    def move(self, name, dof):
        if name == 'pelvis_tilt':
            quat = rpytoQUAT(dof,se3.utils.npToTuple(self.q[1])[0],se3.utils.npToTuple(self.q[2])[0])
            self.q[3] = quat[0]
            self.q[4] = quat[1]
            self.q[5] = quat[2]
            self.q[6] = quat[3]
            self.display(self.q)
        if name == 'pelvis_list':
            quat = rpytoQUAT(se3.utils.npToTuple(self.q[0])[0],dof,se3.utils.npToTuple(self.q[2])[0])
            self.q[3] = quat[0]
            self.q[4] = quat[1]
            self.q[5] = quat[2]
            self.q[6] = quat[3]
            self.display(self.q)
        if name == 'pelvis_rotation':
            quat = rpytoQUAT(se3.utils.npToTuple(self.q[0])[0],se3.utils.npToTuple(self.q[1])[0],dof)
            self.q[3] = quat[0]
            self.q[4] = quat[1]
            self.q[5] = quat[2]
            self.q[6] = quat[3]
            self.display(self.q)
        if name == 'pelvis_tx':
            self.q[0] = dof
            self.display(self.q)
        if name == 'pelvis_ty':
            self.q[1] = dof
            self.display(self.q)
        if name == 'pelvis_tz':
            self.q[2] = dof
            self.display(self.q)
    #POSES
    def zero_poseDisplay(self):
        v = zero(self.model.nv)
        q = zero(self.model.nq)
        q[6] = 1
        self.q = q
        return q

    def half_sitting(self):
        q = self.q0
        q[2] = 0.92#0.81
        v = self.v0
        idx = self.model.getJointId('hip_r')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', 0.2) * rotate('y', -0.05)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        idx = self.model.getJointId('hip_l')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', 0.2) * rotate('y', 0.05)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        idx = self.model.getJointId('knee_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = -0.4#1.22
        idx = self.model.getJointId('knee_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = -0.4
        idx = self.model.getJointId('ankle_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.25#0.61
        idx = self.model.getJointId('ankle_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.25
        idx = self.model.getJointId('mtp_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = -0.05
        idx = self.model.getJointId('mtp_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = -0.05
        # Torso and head
        idx = self.model.getJointId('back')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', -0.2)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        idx = self.model.getJointId('neck')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', 0.2)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        # upper body
        idx = self.model.getJointId('acromial_r')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', -0.2)*rotate('y',-0.18)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        idx = self.model.getJointId('acromial_l')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('x', -0.2)*rotate('y',0.18)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        idx = self.model.getJointId('elbow_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.7
        idx = self.model.getJointId('elbow_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.7
        idx = self.model.getJointId('lunate_hand_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.15
        idx = self.model.getJointId('lunate_hand_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.15
        idx = self.model.getJointId('radioulnar_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = 1.
        idx = self.model.getJointId('radioulnar_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = 1.#0.22
        idx = self.model.getJointId('radius_lunate_r')
        idx = self.model.joints[idx].idx_q
        q[idx] = -0.02
        idx = self.model.getJointId('radius_lunate_l')
        idx = self.model.joints[idx].idx_q
        q[idx] = 0.02
        
        self.q = q
        return q

    # utils
    def SphericalToRPY(joint):
        # i.e. SphericalToRPY('hip_r')
        i = pinocchioRobot.getDoFIdx(joint)  
        quat = np.matrix([ pinocchioRobot.q[i,0], pinocchioRobot.q[i+1,0],                            
                           pinocchioRobot.q[i+2,0], pinocchioRobot.q[i+3,0] ], np.float)  
        quat = np.squeeze(np.asarray(quat))                
        rpy = se3.utils.matrixToRpy(se3.Quaternion(quat[3], quat[0], quat[1], quat[2]).matrix())    
        return rpy 










    
    def showCoM(self, q, segment=None):
        if segment is None:
            pose = self.com(q)
            CoM = se3.SE3.Identity()
            CoM.translation = pose
            self.display.viewer.gui.addXYZaxis('world/sphere', [0., 1., 0., .5], 0.03, 0.3)
            self.display.place("world/sphere", CoM, True)
        elif segment is 'All':
            self.com(q)
            for i in range(0,len(self.data.com)):
                if i == 0:
                    pose = self.data.com[i]
                    CoM = se3.SE3.Identity()
                    CoM.translation = pose
                    #CoM = self.data.oMi[i]*CoM
                    self.display.viewer.gui.addXYZaxis('world/CoM', [0., 0., 1., .8], 0.03, 0.2)
                    self.display.place('world/CoM', CoM, True)
                else:
                    visualName = self.visuals[i][0] 
                    pose = self.model.inertias[i].lever
                    CoM = se3.SE3.Identity()
                    CoM.translation = pose                     
                    CoM = self.data.oMi[i]*CoM
                    self.display.viewer.gui.addXYZaxis('world/'+visualName+'CoM', [0., 1., 0., .5], 0.02, 0.1)
                    self.display.place('world/'+visualName+'CoM', CoM, True)
                
        else:
            print 'each segment'


    def t_poseDisplay(self):
        q = zero(self.model.nq)
        q[6] = 1
        v = zero(self.model.nv)
        # Right Arm
        # This is used to obtain the index of the joint that will be rotated
        idx = self.model.getJointId('shoulder_r')
        idx = self.model.joints[idx].idx_q
        # The shoulder is a spherical joint expressed as quaterion to avoid the singularities of Euler angles
        # We first rotate the DoF in se3
        M = se3.SE3.Identity()
        M.rotation = rotate('y', -np.pi/2)
        # Now we convert it in a quaternion
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]
        
        
        # Rotate left arm
        idx = self.model.getJointId('shoulder_l')
        idx = self.model.joints[idx].idx_q
        M = se3.SE3.Identity()
        M.rotation = rotate('y', np.pi/2)
        Mquat = se3ToXYZQUAT(M)
        q[idx] = Mquat[3]
        q[idx+1] = Mquat[4]
        q[idx+2] = Mquat[5]
        q[idx+3] = Mquat[6]

        # Now the forward dynamics is computed to obtain the T pose
        self.display(q,v)
        self.q = q

        
    #def printSegments(self):
    #    for i in range(0, len(self.model.names)):
    #        print(self.model.names[i])

    def printJoints(self):
        for i in range(0, len(self.model.names)):
            print(self.model.names[i])

    def rotate(self, q, body_name, axis, angle):
        idx = self.getDoFIdx(body_name)
        '''
            Pelvis is a freeflyer joint [0,...,6]
        '''
        if body_name == ('Pelvis_body'):
            self.rotateFFJ(q, axis, angle, idx)
            return
        '''
            The thorax is a spherical joint [7,...,11]
        '''
        if body_name == ('Thorax_body'):
            self.rotateSPHJ(q, axis, angle, idx)
            return
            
        if body_name == 'Head_Neck_body':
            self.rotateSPHJ(q, axis, angle, idx)
            return
        
        if body_name == 'RArm_body':
            self.rotateSPHJ(q, axis, angle, idx)
            return

        if body_name == 'LArm_body':
            self.rotateSPHJ(q, axis, angle, idx)
            return

        # check axis
        if body_name == 'RForearm_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'LForearm_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'RHand_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'LHand_body':
            self.rotateREVJ(q, 'x', angle, idx)    
            return
        
        if body_name == 'RThigh_body':
            self.rotateSPHJ(q, axis, angle, idx)
            return

        if body_name == 'LThigh_body':
            self.rotateSPHJ(q, axis, angle, idx)
            return

        if body_name == 'RShank_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'LShank_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return
        
        if body_name == 'RFoot_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'LFoot_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'HRFingers_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'HLFingers_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return
        
        if body_name == 'FRFingers_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

        if body_name == 'FLFingers_body':
            self.rotateREVJ(q, 'x', angle, idx)
            return

    def play(self, q,v):            
        se3.forwardKinematics(self.model, self.data, q, v)
        displayModel(self.data, self.visuals)
        self.q = q            
        self.showCoM(q,'All')
        
    def rotateFFJ(self, q, axis, angle, idx):
        v = zero(self.model.nv)
        M = se3.SE3.Identity()
        #M.rotation = self.data.oMi[idx].rotation * rotate(axis, angle)
        M.rotation = rotate(axis, angle)
        Mquat = se3ToXYZQUAT(M)
        for dof in range (idx, idx+7):
            q[dof] = Mquat[dof-idx]
        self.play(q,v)
        
    def rotateSPHJ(self, q, axis, angle, idx):
        v = zero(self.model.nv)
        M = se3.SE3.Identity()
        #M.rotation = self.data.oMi[idx].rotation * rotate(axis, angle)
        M.rotation = rotate(axis, angle)
        Mquat = se3ToXYZQUAT(M)
        for dof in range (idx, idx+4):
            q[dof] = Mquat[3+dof-idx]
        self.play(q,v)

    def rotateREVJ(self, q, axis, angle, idx):
            #M = se3.SE3.Identity()
            #M.rotation = self.data.oMi[idx].rotation * rotate(axis, angle)
            #Mquat = se3ToXYZQUAT(M)
        v = zero(self.model.nv)
        q[idx]=angle
        self.play(q,v)
            
        
