'''Code to run SLAM on a physical robot.'''

import os
import time
from math import radians

import slam
from explorer import Explorer

from adafruit_servokit import ServoKit
import board
import busio
import adafruit_vl53l0x

class RobotControl(slam.SensingAndControl):
    '''A physical robot.'''

    def __init__(self):
        super().__init__()
        self.kit = ServoKit(channels=16)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
        self.sensor.measurement_timing_budget = 200000
        self.throttle_trims = [0.0, 0.0, 0.075, 0.09]  # Experimentally determined.

    def get_distance_reading(self):
        '''Returns the distance reading.'''
        self.kit.servo[4].angle = 0
        reading = []
        time.sleep(0.5)

        for i in range(181):
            self.kit.servo[4].angle = i
            if i == 0:
                time.sleep(0.24)
            time.sleep(0.01)
            degrees = (0.67 * i) + 33.6  # This formula is accounting for the servo not being very good.
            rads = radians(degrees - 90)
            reading.append((rads, self.sensor.range + 30))

        self.kit.servo[4].angle = 90
        return reading

    def move(self, theta, distance):
        '''Turn theta and go distance.'''
        self.turn(theta)
        self.move_internal(distance)
        return distance

    def stop(self):
        '''Stop all wheels.'''
        self.kit.continuous_servo[0].fraction = None
        self.kit.continuous_servo[1].fraction = None
        self.kit.continuous_servo[2].fraction = None
        self.kit.continuous_servo[3].fraction = None

    def turn(self, a):
        if a == 0:
            return

        turn_speed = 0.15
        turn_time = a * (0.8 / radians(90))  # Experimentally determined.

        if turn_time < 0:
            turn_time = -turn_time
            turn_speed = -turn_speed - 0.04  # Experimentally determined.

        self.kit.continuous_servo[0].throttle = self.throttle_trims[0]-turn_speed
        self.kit.continuous_servo[1].throttle = self.throttle_trims[1]-turn_speed
        self.kit.continuous_servo[2].throttle = self.throttle_trims[2]-turn_speed
        self.kit.continuous_servo[3].throttle = self.throttle_trims[3]-turn_speed
        time.sleep(turn_time)
        self.stop()

    def move_internal(self, d):
        if d == 0:
            return

        drive_speed = 0.15
        drive_time = d * 0.00375  # Experimentally determined.

        self.kit.continuous_servo[0].throttle = self.throttle_trims[0]-drive_speed  # Front right.
        self.kit.continuous_servo[1].throttle = self.throttle_trims[1]-drive_speed  # Back right.
        self.kit.continuous_servo[2].throttle = self.throttle_trims[2]+drive_speed  # Back left.
        self.kit.continuous_servo[3].throttle = self.throttle_trims[3]+drive_speed  # Front left.
        time.sleep(drive_time)
        self.stop()

if __name__ == '__main__':
    bot_control = RobotControl()

    # Distances measured in mm.
    options = {}
    options[slam.COLLISION_MAP_SCALE] = 100
    options[slam.COLLISION_MAP_MAX_DISTANCE] = 530
    options[slam.EKF_ODOMETRY_NOISE] = 1
    options[slam.EKF_RANGE_NOISE] = 12
    options[slam.EKF_BEARING_NOISE] = radians(5)
    options[slam.EKF_INNOVATION_LAMBDA] = 0.5
    options[slam.EKF_LANDMARK_THRESHOLD] = 7
    options[slam.SPIKE_THRESHOLD] = 400
    options[slam.RANSAC_SAMPLES] = 4
    options[slam.RANSAC_RANGE] = radians(90)
    options[slam.RANSAC_ERROR] = 10
    options[slam.RANSAC_CONSENSUS] = 8
    slam_instance = slam.Slam(bot_control, option_dictionary=options)
    explorer = Explorer(slam_instance, 10)
    explorer.step()

    try:
        for i in range(50):
            print("Working on step {}.".format(i + 1))
            explorer.step()
    except BaseException as e:
        bot_control.stop()
        print(e)

    f = open(os.path.join(".", "results", "robot_collision_map_{}.txt".format(time.strftime("%Y-%m-%d-%H-%M"))), "w")
    f.write(str(slam_instance.get_collision_map()))
    f.close()
