import sys
from math import sin, cos, tan, pi
import board


class Mecanum:
    """ Motor numbering
            m1|  ↑  |m2
              |     |
            m3|     |m4
    Units: velocity: mm/s
            direction: degrees
    """
    def __init__(self, lx=100, ly=50, wheel_diameter=80):
        self.lx = lx   # 0.5 distance between wheel centers- width
        self.ly = ly   # 0.5 distance between wheel centers- length
        self.wheel_diameter = wheel_diameter
        self.velocity = self.direction = self.angular_vel = 0

    def reset(self):
        for i in range(1, 5):
            board.setMotor(i, 0)

        self.velocity = self.direction = self.angular_vel = 0

    def set_velocity(self, velocity, direction, angular_rate):
        """reference mecanum fwd & inverse kinematics formulas"""
        vx = velocity * cos(direction*pi/180)
        vy = velocity * sin(direction*pi/180)
        vp = -angular_rate * (self.lx + self.ly)
        v1 = int(vy + vx - vp)
        v2 = int(vy - vx + vp)
        v3 = int(vy - vx - vp)
        v4 = int(vy + vx + vp)

        # print(v1, v2, v3, v4)

        for motor, speed in zip([1, 2, 3, 4], [v1, v2, v3, v4]):
            board.setMotor(motor, speed)

        self.direction = direction
        self.velocity = velocity
        self.angular_vel = angular_rate

    def set_xy_velocity(self, velo_x, velo_y):
        ...


if __name__ == '__main__':
    m = Mecanum()
    m.set_velocity(50, 180, 0.3)
