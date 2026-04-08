# Import of main files
from env.simulation import Simulation
from controllers.controller import Controller

# Import of main libraries
import pybullet as p
import numpy as np
import random
import time
import os

def main():

    sim = Simulation(gui=True)
    
    controllers = []
    NBR_robot_units = 4
    SIM_DURATION_SECONDS = 20

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

    #Custom start positions for testing
    # chosen_spots = [
    #     [0.25, -0.25, 0.15]]

######################## LOAD ROBOTS AND BALLS ########################
    for pos in chosen_spots:
        random_yaw = random.uniform(0, 2 * np.pi)# Generate a random yaw angle
        #custom random yaw for testing
        #random_yaw = -3*np.pi/4
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

        NBR_balls_per_robot = 0

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

    start_time = time.time()
    video_path = os.path.join(os.path.dirname(__file__), f"simulation_{int(time.time())}.mp4")
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_path)
    print(f"Recording started: {video_path}")

    #pause button 
    pause_button = p.addUserDebugParameter("Pause / Resume", 1, 0, 0)
    previous_clicks = 0
    is_paused = False

    ################ Tracing Initialization ################
    position_history = {}
    prev_delayed_positions = {}
    robot_colors = {}
    trace_delay_seconds = 1.5
    physics_dt = 1.0 / 240.0
    sim_time = 0.0
    trace_palette = [
        [1.0, 0.2, 0.2],  # red
        [0.1, 0.8, 1.0],  # cyan
        [1.0, 0.9, 0.2],  # yellow
        [0.2, 1.0, 0.2],  # green
        [1.0, 0.5, 0.1],  # orange
        [0.9, 0.2, 1.0],  # magenta
    ]
    
    for idx, controller in enumerate(controllers):
        pos, _ = p.getBasePositionAndOrientation(controller.robot)
        initial_pos = [pos[0], pos[1], 0.01]  # Keep traces above z-fighting with the plane
        position_history[controller.robot] = [(sim_time, initial_pos)]
        prev_delayed_positions[controller.robot] = initial_pos
        robot_colors[controller.robot] = trace_palette[idx % len(trace_palette)]
    ######################### SIMULATION LOOP ########################
    try:
        while True:
            if time.time() - start_time >= SIM_DURATION_SECONDS:
                print(f"Simulation finished after {SIM_DURATION_SECONDS} seconds.")
                break

            ################Pause functionality################
            current_clicks = p.readUserDebugParameter(pause_button)
            if current_clicks > previous_clicks:
                is_paused = not is_paused        
                previous_clicks = current_clicks 

            if is_paused:
                time.sleep(0.01)
                continue
            
            #########FSM######### 
            # State 1: Move forward
            # State 2: Turn left
            # State 3: Turn right
            # State 4 : Stop
            # Transition: If front sensor detects obstacle, turn left. If left sensor detects obstacle,
            
            # sensor input to be added : how can we detect the ball bellow?
            # Simple fix: Use p.getContactPoints(robot_id, ball_id) or p.rayTest() here.

            
            
            for controller in controllers:
                controller.random_fsm_step(speed=10)
                # controller.nn_obstacle_avoidance_step(
                #     forward_speed=8,
                #     max_speed=16,
                #     sensor_length=0.22,
                #     sensor_angle_deg=55,
                #     mirror_turn=True,
                # )

                sim.step()
                sim_time += physics_dt

                ####################### TRACING #######################
                current_pos, _ = p.getBasePositionAndOrientation(controller.robot)
                current_pos_ground = [current_pos[0], current_pos[1], 0.01]
                history = position_history[controller.robot]

                # Store current robot trajectory over time.
                last_recorded_pos = history[-1][1]
                dist_now = (current_pos_ground[0] - last_recorded_pos[0])**2 + (current_pos_ground[1] - last_recorded_pos[1])**2
                if dist_now > 0.00001:  # Update roughly every 1mm
                    history.append((sim_time, current_pos_ground))

                target_time = sim_time - trace_delay_seconds

                # Keep only the interval that brackets target_time.
                while len(history) >= 2 and history[1][0] <= target_time:
                    history.pop(0)

                delayed_pos = history[0][1]
                if len(history) >= 2 and history[0][0] <= target_time <= history[1][0]:
                    t0, p0 = history[0]
                    t1, p1 = history[1]
                    denom = (t1 - t0)
                    alpha = 0.0 if denom <= 1e-9 else (target_time - t0) / denom
                    delayed_pos = [
                        p0[0] + alpha * (p1[0] - p0[0]),
                        p0[1] + alpha * (p1[1] - p0[1]),
                        0.01
                    ]

                prev_delayed = prev_delayed_positions[controller.robot]
                dist_delayed = (delayed_pos[0] - prev_delayed[0])**2 + (delayed_pos[1] - prev_delayed[1])**2
                if dist_delayed > 0.000001:
                    p.addUserDebugLine(
                        prev_delayed,
                        delayed_pos,
                        lineColorRGB=robot_colors[controller.robot],
                        lineWidth=6,
                        lifeTime=0
                    )
                    prev_delayed_positions[controller.robot] = delayed_pos

                ####################### For now : simple forward movement #######################
                # controller.turn_left(speed=10) # 16 is the speed, can be tuned

    finally:
        p.stopStateLogging(log_id)
        print(f"Recording saved: {video_path}")

if __name__ == "__main__":
    main()