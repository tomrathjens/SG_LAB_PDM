import pybullet as p

class Arena: #arena size SGlab = 2.1x2.1x0.3
    def __init__(self, size=0.8, wall_height=0.1, wall_thickness=0.1):#in meters
        self.size = size
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.create_ground()
        self.create_walls()

    def create_ground(self):
        ground_shape = p.createCollisionShape(p.GEOM_PLANE)
        self.ground_id = p.createMultiBody(0, ground_shape)
        p.changeDynamics(self.ground_id, -1, lateralFriction=2.0, rollingFriction=0.0, spinningFriction=0.0, restitution=0.5)
    def create_walls(self):
        s = self.size
        h = self.wall_height
        t = self.wall_thickness

        # +x wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                halfExtents=[t/2, s/2, h/2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                halfExtents=[t/2, s/2, h/2],
                rgbaColor = [0.5, 0, 0.5, 1]),
            basePosition=[s/2, 0, h/2]
        )
        # -x wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                halfExtents=[t/2, s/2, h/2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                halfExtents=[t/2, s/2, h/2],
                rgbaColor = [0.5, 0, 0.5, 1]),
            basePosition=[-s/2, 0, h/2]
        )
        # +y wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                halfExtents=[s/2, t/2, h/2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                halfExtents=[s/2, t/2, h/2],
                rgbaColor = [0.5, 0, 0.5, 1]),
            basePosition=[0, s/2, h/2]
        )
        # -y wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX,
                halfExtents=[s/2, t/2, h/2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                halfExtents=[s/2, t/2, h/2],
                rgbaColor = [0.5, 0, 0.5, 1]),
            basePosition=[0, -s/2, h/2]
        )