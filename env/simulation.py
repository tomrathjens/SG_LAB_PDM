#Here, we create the simulations world.

#librairies
import pybullet as p
import pybullet_data #contains example assets (plane, robots, textures)
import time

#arena import
from env.arena import Arena

class Simulation:
    def __init__(self, gui=True):#set gui to True to see the simulation
        if gui:
            self.client = p.connect(p.GUI)#to create a graphical interface
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # clean GUI

            p.resetDebugVisualizerCamera(cameraDistance=1.5,cameraYaw=50,cameraPitch=-35,cameraTargetPosition=[0,0,0])
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #to tell pybullet where to find the assets (here within the public librairy)
        p.setGravity(0, 0, -9.81)

        #self.plane = p.loadURDF("plane.urdf")
        self.arena = Arena()
        

    def load_robot(self, urdf_path, start_pos=[0,0,0.5], start_orientation=[0,0,0]):

        start_orientation = p.getQuaternionFromEuler([start_orientation[0], start_orientation[1], start_orientation[2]])

        self.robot = p.loadURDF(
            urdf_path,
            start_pos,
            start_orientation
        )

        # wheel friction
        p.changeDynamics(self.robot, 0, lateralFriction=1.0)
        p.changeDynamics(self.robot, 1, lateralFriction=1.0)

    def step(self): #perform one physical step, where it computes forces, collisions ect... and updates the state of the simulation
        p.stepSimulation()
        time.sleep(1./240.) #240 Hz default for simulation

    def disconnect(self):
        p.disconnect()