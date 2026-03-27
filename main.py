# Import of main files
from env.simulation import Simulation
from controllers.controller import Controller

# Import of main libraries
import pybullet as p
import numpy as np
import random

def main():

    sim = Simulation(gui=True)
    controllers = []
    NBR_robot_units = 20

    ################### list of possible start positions for the robots ###################
    # arena is 2.1m x 2.1
    grid_x = np.arange(-0.9, 0.9, 0.15)
    grid_y = np.arange(-0.9, 0.9, 0.15)
    all_safe_spots = [[x, y, 0.025] for x in grid_x for y in grid_y]
    chosen_spots = random.sample(all_safe_spots, NBR_robot_units)

######################## LOAD ROBOTS AND BALLS ########################
    for pos in chosen_spots:
        random_yaw = random.uniform(0, 2 * np.pi)# Generate a random yaw angle
        #####################load the robot##################
        sim.load_robot(
            "robots/full_assembly_robot_unit.urdf",
            start_pos=pos,
            start_orientation=[-np.pi/2, 0, random_yaw] # roll, pitch, yaw
        )
        robot_id = sim.robot
        controllers.append(Controller(robot_id))
        #p.changeDynamics(robot_id, -1, contactMargin=0.0001)

        #########################load the balls in the reservoir########################
        ball_size = 0.005
        Old_reservoir_coord = np.array([0, -0.014, 0])

        R = np.array([
            [np.cos(random_yaw), -np.sin(random_yaw), 0],
            [np.sin(random_yaw),  np.cos(random_yaw), 0],
            [0, 0, 1]
        ])#rotation matrix bcause random yaw

        Rotated_reservoir_coord = R @ Old_reservoir_coord

        NBR_balls_per_robot = 20

        for i in range(NBR_balls_per_robot):
            ball_pos = [
                pos[0] + Rotated_reservoir_coord[0],
                pos[1] + Rotated_reservoir_coord[1],
                pos[2] + Rotated_reservoir_coord[2] + ball_size * i
            ]

            ball_id = p.loadURDF("robots/ball1.urdf", basePosition=ball_pos)
            p.changeDynamics(
                ball_id,
                -1,
                contactStiffness=1e4,  # Higher stiffness for tiny objects
                contactDamping=100,linearDamping=0.0, angularDamping=0.0
            )


######################## GUI CONTROLS ########################
    #pause button 
    pause_button = p.addUserDebugParameter("Pause / Resume", 1, 0, 0)
    previous_clicks = 0
    is_paused = False

    ######################### SIMULATION LOOP ########################
    while True:
        ################Pause functionality################
        current_clicks = p.readUserDebugParameter(pause_button)
        if current_clicks > previous_clicks:
            is_paused = not is_paused        
            previous_clicks = current_clicks 
        
        ############### Locomotion strategy ###############
        if not is_paused:
            #########FSM######### 
                #State 1: Move forward
                #State 2: Turn left
                #State 3: Turn right
                #State 4 : Stop
                #Transition: If front sensor detects obstacle, turn left. If left sensor detects obstacle,
                #sensor input to be added : how can we detect the ball bellow?

            for controller in controllers:
                controller.forward(16) #8 is the speed, can be tuned
                
            sim.step()

if __name__ == "__main__":
    main()