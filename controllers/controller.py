import pybullet as p
import random
import math
import time

class Controller:

    def __init__(self, robot_id):
        self.robot = robot_id

        self.left_wheel = 0
        self.right_wheel = 1
        self.drive_force = 0.25
        
        # Give each controller its own state memory
        self.current_state = "forward"
        self.turn_until = 0.0
        self.turn_direction = "left"

    def stop(self, speed=0):
        p.setJointMotorControl2(
            self.robot, self.left_wheel,
            p.VELOCITY_CONTROL, targetVelocity=0, force=self.drive_force
        )
        p.setJointMotorControl2(
            self.robot, self.right_wheel,
            p.VELOCITY_CONTROL, targetVelocity=0, force=self.drive_force
        )

    def forward(self, speed=10):

        p.setJointMotorControl2(
            self.robot,
            self.left_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=-speed,
            force=self.drive_force
        )

        p.setJointMotorControl2(
            self.robot,
            self.right_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=-speed,
            force=self.drive_force
        )

    def turn_left(self, speed=5):

        p.setJointMotorControl2(self.robot, self.left_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=speed, force=self.drive_force)

        p.setJointMotorControl2(self.robot, self.right_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=-speed, force=self.drive_force)
        
    def turn_right(self, speed=5):

        p.setJointMotorControl2(self.robot, self.left_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=-speed, force=self.drive_force)

        p.setJointMotorControl2(self.robot, self.right_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=+speed, force=self.drive_force)

    def random_fsm_step(self, speed=10):
        potential_new_state = random.choices(
            population=["forward", "turn_left", "turn_right", "stop"],
            weights=[0.5, 0.05, 0.05, 0.001]
        )[0]

        state_selection = random.choices(
            population=[self.current_state, potential_new_state],
            weights=[0.99, 0.01],
            k=1
        )[0]

        self.current_state = state_selection

        if hasattr(self, state_selection):
            action = getattr(self, state_selection)
            action(speed=speed)

    def nn_obstacle_avoidance_step(
        self,
        forward_speed=8,
        max_speed=16,
        sensor_length=0.22,
        sensor_angle_deg=55,
        mirror_turn=True,
    ):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(base_orn)[2]
        sensor_angle = math.radians(sensor_angle_deg)

        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        ray_start = [
            base_pos[0] + 0.02 * fwd_x,
            base_pos[1] + 0.02 * fwd_y,
            0.02,
        ]

        def ray_distance(relative_angle):
            ray_dir_x = math.cos(yaw + relative_angle)
            ray_dir_y = math.sin(yaw + relative_angle)
            ray_end = [
                ray_start[0] + sensor_length * ray_dir_x,
                ray_start[1] + sensor_length * ray_dir_y,
                0.02,
            ]
            hit = p.rayTest(ray_start, ray_end)[0]
            hit_body_id = hit[0]
            hit_fraction = hit[2]
            if hit_body_id < 0 or hit_body_id == self.robot:
                return sensor_length
            return hit_fraction * sensor_length

        front_dist = ray_distance(0.0)
        left_dist = ray_distance(sensor_angle)
        right_dist = ray_distance(-sensor_angle)

        # Inputs are obstacle threat levels in [0, 1].
        x_front = max(0.0, min(1.0, 1.0 - front_dist / sensor_length))
        x_left = max(0.0, min(1.0, 1.0 - left_dist / sensor_length))
        x_right = max(0.0, min(1.0, 1.0 - right_dist / sensor_length))

        # Tiny fixed NN: 3 inputs -> 4 hidden tanh units -> 2 outputs.
        h0 = math.tanh(2.2 * x_front - 0.9)
        h1 = math.tanh(2.0 * x_left - 0.6)
        h2 = math.tanh(2.0 * x_right - 0.6)
        h3 = math.tanh(1.4 * x_front + 0.9 * x_left + 0.9 * x_right - 1.2)

        turn = 1.6 * h1 - 1.6 * h2
        slow = max(0.0, min(1.0, 0.8 * h0 + 0.7 * h3))
        if mirror_turn:
            turn = -turn

        speed_scale = max(0.15, 1.0 - 0.8 * slow)
        base = forward_speed * speed_scale
        steer_gain = 6.0

        left_cmd = -base + steer_gain * turn
        right_cmd = -base - steer_gain * turn

        left_cmd = max(-max_speed, min(max_speed, left_cmd))
        right_cmd = max(-max_speed, min(max_speed, right_cmd))

        p.setJointMotorControl2(
            self.robot,
            self.left_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=left_cmd,
            force=self.drive_force,
        )
        p.setJointMotorControl2(
            self.robot,
            self.right_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=right_cmd,
            force=self.drive_force,
        )

    def obstacle_avoidance_step(
        self,
        forward_speed=6,
        turn_speed=14,
        sensor_length=0.22,
        avoid_distance=0.16,
        side_margin=0.13,
        sensor_angle_deg=55,
        turn_commit_seconds=0.35,
    ):
        # On this robot, wheel/joint orientation makes logical left/right commands appear mirrored.
        def avoid_turn(direction, speed):
            if direction == "left":
                self.turn_right(speed=speed)
            else:
                self.turn_left(speed=speed)

        t = time.time()
        if t < self.turn_until:
            avoid_turn(self.turn_direction, turn_speed)
            return

        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        yaw = p.getEulerFromQuaternion(base_orn)[2]
        sensor_angle = math.radians(sensor_angle_deg)

        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        ray_start = [
            base_pos[0] + 0.02 * fwd_x,
            base_pos[1] + 0.02 * fwd_y,
            0.02,
        ]

        def ray_distance(relative_angle):
            ray_dir_x = math.cos(yaw + relative_angle)
            ray_dir_y = math.sin(yaw + relative_angle)
            ray_end = [
                ray_start[0] + sensor_length * ray_dir_x,
                ray_start[1] + sensor_length * ray_dir_y,
                0.02,
            ]
            hit = p.rayTest(ray_start, ray_end)[0]
            hit_body_id = hit[0]
            hit_fraction = hit[2]
            if hit_body_id < 0 or hit_body_id == self.robot:
                return sensor_length
            return hit_fraction * sensor_length

        front_dist = ray_distance(0.0)
        left_dist = ray_distance(sensor_angle)
        right_dist = ray_distance(-sensor_angle)

        if front_dist < avoid_distance * 0.60:
            # Emergency: rotate in place for a short, committed burst.
            if left_dist >= right_dist:
                self.turn_direction = "left"
                avoid_turn("left", turn_speed)
            else:
                self.turn_direction = "right"
                avoid_turn("right", turn_speed)
            self.turn_until = t + turn_commit_seconds * 1.5
        elif front_dist < avoid_distance:
            if left_dist >= right_dist:
                self.turn_direction = "left"
                avoid_turn("left", turn_speed)
            else:
                self.turn_direction = "right"
                avoid_turn("right", turn_speed)
            self.turn_until = t + turn_commit_seconds
        elif left_dist < side_margin:
            avoid_turn("right", turn_speed)
        elif right_dist < side_margin:
            avoid_turn("left", turn_speed)
        else:
            self.forward(speed=forward_speed)