from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait


FRONT_ARM_GEAR_RATIO = [[28,20]]  # output degrees per motor degree
REAR_ARM_GEAR_RATIO  = [[12,36], [12,20]]

hub = PrimeHub()
left_motor  = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
front_motor = Motor(Port.C, gears=FRONT_ARM_GEAR_RATIO, reset_angle=False) # arms
rear_motor  = Motor(Port.D, Direction.COUNTERCLOCKWISE, gears=REAR_ARM_GEAR_RATIO, reset_angle=False) # whacker

front_motor.reset_angle(0)
rear_motor.reset_angle(155)


while True:
    print("left_motor:", left_motor.angle(), "  right_motor:", right_motor.angle(), "  front_motor:", front_motor.angle(), "  rear_motor:", rear_motor.angle(), "  heading:", hub.imu.heading())
    wait(100)