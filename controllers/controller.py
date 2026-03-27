import pybullet as p

class Controller:

    def __init__(self, robot_id):
        self.robot = robot_id

        self.left_wheel = 0
        self.right_wheel = 1
        
        # Give each controller its own state memory
        self.current_state = "forward"

    def stop(self, speed=0):
        p.setJointMotorControl2(
            self.robot, self.left_wheel,
            p.VELOCITY_CONTROL, targetVelocity=0, force=0.05
        )
        p.setJointMotorControl2(
            self.robot, self.right_wheel,
            p.VELOCITY_CONTROL, targetVelocity=0, force=0.05
        )

    def forward(self, speed=10):

        p.setJointMotorControl2(
            self.robot,
            self.left_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=-speed,
            force =0.05
        )

        p.setJointMotorControl2(
            self.robot,
            self.right_wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=-speed,
            force =0.05
        )

    def turn_left(self, speed=5):

        p.setJointMotorControl2(self.robot, self.left_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=speed, force =0.05)

        p.setJointMotorControl2(self.robot, self.right_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=-speed, force =0.05)
        
    def turn_right(self, speed=5):

        p.setJointMotorControl2(self.robot, self.left_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=-speed, force =0.05)

        p.setJointMotorControl2(self.robot, self.right_wheel,
                                p.VELOCITY_CONTROL, targetVelocity=+speed, force =0.05)