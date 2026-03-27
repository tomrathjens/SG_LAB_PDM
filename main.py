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
    NBR_robot_units = 6


    ################### list of possible start positions for the robots ###################
    # arena is 2.1m x 2.1
    #get arena size from the arena class
    arena_size = sim.arena.size
    grid_x = np.arange(-arena_size/2 + 0.15, arena_size/2 - 0.15, 0.15)
    grid_y = np.arange(-arena_size/2 + 0.15, arena_size/2 - 0.15, 0.15)
    # grid_x = np.arange(-0.9, 0.9, 0.15)
    # grid_y = np.arange(-0.9, 0.9, 0.15)
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
        ball_size = 0.006
        Old_reservoir_coord = np.array([0, -0.014, 0])

        R = np.array([
            [np.cos(random_yaw), -np.sin(random_yaw), 0],
            [np.sin(random_yaw),  np.cos(random_yaw), 0],
            [0, 0, 1]
        ])#rotation matrix bcause random yaw

        Rotated_reservoir_coord = R @ Old_reservoir_coord

        NBR_balls_per_robot = 15

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
                lateralFriction=0.2,       # Smooth, hard surface friction
                rollingFriction=0.0005,    # Extremely low rolling drag (rolls very freely)
                spinningFriction=0.0005,   # Easily pivots
                restitution=0.7,           # Bouncy (hard plastic/glass)
                linearDamping=0.0,         # Zero artificial aerodynamic drag
                angularDamping=0.0         # Zero artificial rotational drag
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
            # State 1: Move forward
            # State 2: Turn left
            # State 3: Turn right
            # State 4 : Stop
            # Transition: If front sensor detects obstacle, turn left. If left sensor detects obstacle,
            
            # sensor input to be added : how can we detect the ball bellow?
            # Simple fix: Use p.getContactPoints(robot_id, ball_id) or p.rayTest() here.

            
            
            for controller in controllers:
                # ####################### For now : random movement, based on previous state #######################
                potential_new_state = random.choices(
                    population=["forward", "turn_left", "turn_right", "stop"],
                    weights=[0.5, 0.2, 0.2, 0.1]
                )[0] 
                
                state_selection = random.choices(
                    population=[controller.current_state, potential_new_state],
                    weights=[0.99, 0.01],
                    k=1
                )[0]

                # Update the specific controller's state memory
                controller.current_state = state_selection
                
                # Dynamically call the method (forward, turn_left, etc.) based on the specific controller's state
                if hasattr(controller, state_selection):
                    action = getattr(controller, state_selection)
                    action(speed=10) # 10 is the speed, can be tuned

                ####################### For now : simple forward movement #######################
                # controller.turn_left(speed=10) # 16 is the speed, can be tuned
                
            sim.step()

if __name__ == "__main__":
    main()