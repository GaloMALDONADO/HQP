import gepetto.corbaserver
from first_order_low_pass_filter import FirstOrderLowPassFilter
from hqp.wrapper import Wrapper as RobotWrapper
from pinocchio.utils import zero as mat_zeros
import numpy as np
import pinocchio as se3
import os
from pinocchio.utils import XYZQUATToViewerConfiguration, zero, se3ToXYZQUAT

ENABLE_VIEWER = "ON"

def xyzRpyToViewerConfig(xyz, rpy):
    xyz = np.asmatrix(xyz).reshape((3,1))
    rpy = np.asmatrix(rpy).reshape((3,1))
    R = se3.utils.rpyToMatrix(rpy)
    H = se3.SE3(R, xyz)
    pinocchioConf = se3.utils.se3ToXYZQUAT(H)
    return se3.utils.XYZQUATToViewerConfiguration(pinocchioConf)


class Viewer(object):
    COM_SPHERE_RADIUS = 0.01;
    COM_SPHERE_COLOR = (1, 0, 0, 1);
    PLAYER_FRAME_RATE = 20;
    
    CAMERA_LOW_PASS_FILTER_CUT_FREQUENCY = 10.0;
    CAMERA_FOLLOW_ROBOT = '';   # name of the robot to follow with the camera
    CAMERA_LOOK_AT_HEIGHT = 0.5;           # height the camera should look at when following a robot
    CAMERA_REL_POS = [4, 2, 1.75];  # distance from robot to camera
    CAMERA_LOOK_AT_OFFSET = [0, 0]; # offset to robot xy position looked by camera
    
    def __init__(self, name, robot=None):
        if robot is None:
            pass
        self.name = name;
        self.filter = FirstOrderLowPassFilter(0.002, self.CAMERA_LOW_PASS_FILTER_CUT_FREQUENCY, mat_zeros(2))
        self.robot = robot
        self.initDisplay("world/pinocchio")
        nodeName = "world/"+self.robot.name
        self.loadDisplayModel(nodeName, "pinocchio", self.robot)
        self.viewer.gui.setLightingMode('world/floor', 'OFF')
        self.robots = {robot.name: robot};                

    def initDisplay(self, viewerRootNodeName="world/pinocchio"):
        try:
            self.viewer=gepetto.corbaserver.Client()
            print "Connected to corba server"
            self.viewerRootNodeName = viewerRootNodeName
        except:
            if 'viewer' in self.__dict__:
                del self.viewer
                print "!! Error while starting the viewer client. "
                print "Check whether Gepetto-viewer-corba is properly started"

    # Create the scene displaying the robot meshes in gepetto-viewer      
    def loadDisplayModel(self, nodeName, windowName, robot):
        # Open a window for displaying your model.     
        try:
            # If the window already exists, do not do anything
            self.windowID = self.viewer.gui.getWindowID (windowName)
            print "Warning: window '"+windowName+"' already created."
            print "The previously created objects will not be destroyed and do not have to be created again."
        except:
            # Otherwise, create the empty window.
            self.windowID = self.viewer.gui.createWindow (windowName)
        print robot.name
        # Start a new "scene" in this window, named "world", with just a floor.
        if "world/"+robot.name not in self.viewer.gui.getSceneList():
            self.viewer.gui.createSceneWithFloor("world/"+robot.name)
         
        self.viewer.gui.addSceneToWindow("world/"+robot.name, self.windowID)

        self.viewer.gui.createGroup(nodeName)
        # iterate over visuals and create the meshes in the viewer 
        for i in range (1,len(robot.visuals)):
            self.viewer.gui.addMesh('world/'+robot.name+'/'+
                                    robot.visuals[i][1]+
                                    os.path.split(self.robot.visuals[i][2])[1], 
                                    robot.visuals[i][2])
        # iterate for creating nodes for all joint 
        for i in range(1,robot.model.nbodies):
            self.viewer.gui.addXYZaxis('world/'+robot.name+'/'+
                                       robot.model.names[i], [1., 0., 0., .5], 0.02, 1)
        # create a node for the center of mass
        self.viewer.gui.addXYZaxis('world/'+robot.name+
                                   '/globalCoM', [0., 1., 0., .5], 0.03, 0.3)
        # Finally, refresh the layout to obtain your first rendering
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

    def display(self, q, robotName, osimref=True, com=True, updateKinematics=True):
        robot = self.robots[robotName]
        oMp = se3.SE3.Identity().rotation
        if osimref is True:
            oMp = se3.utils.rotate('z',np.pi/2) * se3.utils.rotate('x',np.pi/2)
        # update q kinematics
        #self.update(q)
        # show CoM
        if com is True:
            CoM = se3.SE3.Identity()
            CoM.translation =robot.com(q)
            self.placeObject('world/'+robot.name+"/globalCoM", CoM, True)

        #display skelette visuals
        for i in range (1,len(robot.visuals)):
            # get the parent joint
            idx = robot.model.getJointId(robot.visuals[i][1])
            pose = se3.SE3.Identity()
            if osimref is True:
                # convert bones pose
                pose.translation =  (robot.data.oMi[idx].translation + 
                                     np.matrix(np.array([0.,0.,0.]) *
                                               np.array(np.squeeze(robot.visuals[i][4][3:6]))[0]).T)
                #*self.visuals[i][4][3:6])
                pose.rotation= robot.data.oMi[idx].rotation * oMp

            else:
                pose.translation= robot.data.oMi[idx].translation+robot.visuals[i][4][3:6]
                pose.rotation= robot.data.oMi[idx].rotation

            # place objects
            if i != len(robot.visuals)-1:
                self.viewer.gui.setScale('world/'+robot.name+'/'+
                                         robot.visuals[i][1]+
                                         os.path.split(robot.visuals[i][2])[1], 
                                         robot.visuals[i][3])
                self.placeObject('world/'+robot.name+'/'+
                                 robot.visuals[i][1]+
                                 os.path.split(robot.visuals[i][2])[1], pose, False)
            else:
                self.viewer.gui.setScale('world/'+robot.name+'/'+
                                         robot.visuals[i][1]+
                                         os.path.split(robot.visuals[i][2])[1], 
                                         robot.visuals[i][3])
                self.placeObject('world/'+robot.name+'/'
                                 +robot.visuals[i][1]+
                                 os.path.split(robot.visuals[i][2])[1], pose, True)

    def updateRobotConfig(self, q, robotName, osimref=True):
        #self.robots[robotName].display(q, robotName)
        '''
        se3.computeAllTerms(self.robots[robotName].model, 
                            self.robots[robotName].data, 
                            q, 
                            self.robots[robotName].v)
        '''
        se3.forwardKinematics(self.robots[robotName].model, 
                              self.robots[robotName].data, 
                              q)
        se3.framesKinematics(self.robots[robotName].model, 
                             self.robots[robotName].data, 
                             q)
        self.display(q,robotName)

    def addRobot(self, robot):
        if(ENABLE_VIEWER):
            self.robots[robot.name] = robot
            nodeName = "world/"+robot.name
            self.loadDisplayModel(nodeName, "pinocchio", self.robots[robot.name]);
            se3.framesKinematics(self.robots[robot.name].model, 
                                 self.robots[robot.name].data, 
                                 self.robots[robot.name].q0)
            self.updateRobotConfig(self.robots[robot.name].q0,robot.name)
            
            

    def addSphere(self,name, radius, xyz, rpy=mat_zeros(3), color=(0,0,0,1.0), lightingMode='ON'):
        #if(ENABLE_VIEWER):
        position = xyzRpyToViewerConfig(xyz, rpy);
        self.viewer.gui.addSphere('world/'+name, radius, color);
        self.viewer.gui.applyConfiguration('world/'+name, position)
        self.viewer.gui.setLightingMode('world/'+name, lightingMode);
        self.viewer.gui.refresh();

    def addLine(self,name, pos1, pos2, color=(0,0,0,1.0), lightingMode='ON'):
        if(ENABLE_VIEWER):
            if(len(pos1.shape)==1):
                self.viewer.gui.addLine(name, 
                                         (pos1[0], pos1[1], pos1[2]), 
                                         (pos2[0], pos2[1], pos2[2]), 
                                         color)
            else:
                self.viewer.gui.addLine(name, 
                                        (pos1[0,0], pos1[1,0], pos1[2,0]), 
                                        (pos2[0,0], pos2[1,0], pos2[2,0]), 
                                        color);
            self.viewer.gui.setLightingMode(name, lightingMode);

    def setVisibility(self, name, mode='OFF'):
        if(ENABLE_VIEWER):
            self.viewer.gui.setVisibility('world/'+name, mode);


