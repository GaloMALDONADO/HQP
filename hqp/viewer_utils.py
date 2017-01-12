from first_order_low_pass_filter import FirstOrderLowPassFilter
from hqp.wrapper import Wrapper as RobotWrapper
from pinocchio.utils import zero as mat_zeros
import numpy as np
import pinocchio as se3
import os

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
    
    def __init__(self, name, robotWrapper, robotName='robot1'):
        self.name = name;
        self.filter = FirstOrderLowPassFilter(0.002, self.CAMERA_LOW_PASS_FILTER_CUT_FREQUENCY, mat_zeros(2));
        self.robot = robotWrapper;
        self.robot.initDisplay("world/"+robotName, loadModel=False);
        self.robot.loadDisplayModel("world/"+robotName);
        self.robot.viewer.gui.setLightingMode('world/floor', 'OFF');
        self.robots = {robotName: self.robot};                
    
    def placeObject(self, objName, M, refresh=True):
        '''                                                                                       
        This function places (ie changes both translation and rotation) of the object names       
        "objName" in place given by the SE3 object "M". By default, immediately refresh the       
        layout. If multiple objects have to be placed at the same time, do the refresh only       
        at the end of the list                                                                    
        '''
        pinocchioConf = se3.utils.XYZQUATToViewerConfiguration(se3.utils.se3ToXYZQUAT(M))
        self.robot.viewer.gui.applyConfiguration(objName,pinocchioConf)
        if refresh: self.robot.viewer.gui.refresh()

    def updateRobotConfig(self, q, robotName, osimref=True):
        self.robots[robotName].display(q);

    def addRobot(self, robot):
        if(ENABLE_VIEWER):
            newRobot = robot
            newRobot.initDisplay("world/"+robot.name, loadModel=False);
            newRobot.loadDisplayModel("world/"+robot.name);
            #newRobot.viewer.gui.addURDF("world/"+robotName, modelPath, meshPath);
            self.robots[robot.name] = newRobot
            

    def addSphere(self,name, radius, xyz, rpy=mat_zeros(3), color=(0,0,0,1.0), lightingMode='ON'):
        #if(ENABLE_VIEWER):
        position = xyzRpyToViewerConfig(xyz, rpy);
        self.robot.viewer.gui.addSphere('world/'+name, radius, color);
        self.robot.viewer.gui.applyConfiguration('world/'+name, position)
        self.robot.viewer.gui.setLightingMode('world/'+name, lightingMode);
        self.robot.viewer.gui.refresh();

    def addLine(self,name, pos1, pos2, color=(0,0,0,1.0), lightingMode='ON'):
        if(ENABLE_VIEWER):
            if(len(pos1.shape)==1):
                self.robot.viewer.gui.addLine('world/'+name, (pos1[0], pos1[1], pos1[2]), (pos2[0], pos2[1], pos2[2]), color);
            else:
                self.robot.viewer.gui.addLine('world/'+name, (pos1[0,0], pos1[1,0], pos1[2,0]), (pos2[0,0], pos2[1,0], pos2[2,0]), color);
            self.robot.viewer.gui.setLightingMode('world/'+name, lightingMode);

    def setVisibility(self, name, mode='OFF'):
        if(ENABLE_VIEWER):
            self.robot.viewer.gui.setVisibility('world/'+name, mode);
