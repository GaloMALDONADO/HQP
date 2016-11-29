from first_order_low_pass_filter import FirstOrderLowPassFilter
import wrapper as RobotWrapper

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
        self.robot.loadDisplayModel("world/"+robotName, robotName);
        self.robot.viewer.gui.setLightingMode('world/floor', 'OFF');
        self.robots = {robotName: self.robot};                
    

    def addRobot(self, robotName, modelPath):
        if(ENABLE_VIEWER):
            newRobot = RobotWrapper(modelPath);
            newRobot.initDisplay("world/"+robotName, loadModel=False);

            newRobot.viewer.gui.addURDF("world/"+robotName, urdfModelPath, modelPath);
            self.robots[robotName] = newRobot;
