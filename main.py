#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# ev3.speaker.beep()
# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
tool_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)
distance_sensor = UltrasonicSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=95)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 2
WHITE = 16
threshold = (BLACK + WHITE) / 2 + 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

# Start following the line endlessly.
start = time.time()
end = time.time()
last_deviation = line_sensor.reflection() - threshold
corner = 1
FIRE_DISTANCE = 75

def putOff(dtc_to_fire):
    robot.turn(90)
    robot.straight(dtc_to_fire)
    for i in range(3):
        tool_motor.run_angle(500, -90)
        tool_motor.run_angle(500,90)
    robot.straight(-dtc_to_fire)
    robot.turn(-90) 

while True:
    wait(10)
    deviation = line_sensor.reflection() - threshold
    v_deviation = (deviation - last_deviation) / 10
    last_deviation = deviation
    if v_deviation <= -0.2:
        if corner >= 3:
            robot.drive(DRIVE_SPEED*0.25, 120)
        else:
            robot.drive(DRIVE_SPEED*0.25, -90)
        corner+=1
        while True:
            wait(10)
            if line_sensor.reflection() >= 12:
                wait(15)
                if corner == 3:
                    robot.stop()
                    wait(2000)
                    ev3.speaker.beep()
                    SoundFile.KUNG_FU.play()
                    robot.straight(100)
                break
        last_deviation = line_sensor.reflection() - threshold
        continue

    if corner > 3 and distance_sensor.distance() < FIRE_DISTANCE:
        robot.stop()
        putOff(distance_sensor.distance())

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation + (3 if deviation > 0 else -3)

    robot.drive(DRIVE_SPEED, turn_rate)