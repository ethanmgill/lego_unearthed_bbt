from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

hub = PrimeHub()
left_motor  = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
front_motor = Motor(Port.C) # arms
rear_motor  = Motor(Port.D) # whacker

while True:
    print("left_motor:", left_motor.angle(), "  right_motor:", right_motor.angle(), "  front_motor:", front_motor.angle(), "  rear_motor:", rear_motor.angle(), "  heading:", hub.imu.heading())
    wait(100)