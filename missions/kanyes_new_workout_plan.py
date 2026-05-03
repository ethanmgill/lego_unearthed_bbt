from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

# ── Robot configuration ───────────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 88    # your wheels
AXLE_TRACK_MM = 145       # axle center-to-center distance
STRAIGHT_SPEED = 300      # mm/s
STRAIGHT_ACCEL = 600      # mm/s²
TURN_RATE = 150           # deg/s
TURN_ACCEL = 300          # deg/s²
GYRO_KP = 1.2             # proportional gain for heading correction

# ── Hardware init ─────────────────────────────────────────────────────────────
hub = PrimeHub()
left_motor  = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
front_motor = Motor(Port.C) # arms
rear_motor  = Motor(Port.D) # whacker
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
robot.settings(STRAIGHT_SPEED, STRAIGHT_ACCEL, TURN_RATE, TURN_ACCEL) # set default speeds and accelerations

# ── Odometry helpers ──────────────────────────────────────────────────────────
def reset_odometry():
    robot.reset()

def get_position():
    """Returns (distance_mm, heading_deg) from last reset."""
    return robot.state()

# ── Gyro-corrected movement ───────────────────────────────────────────────────
def drive_straight(distance_mm, speed=STRAIGHT_SPEED):
    robot.reset()
    target_heading = hub.imu.heading()
    direction = 1 if distance_mm >= 0 else -1
    target_distance = abs(distance_mm)

    while abs(robot.distance()) < target_distance:
        heading_error = hub.imu.heading() - target_heading
        robot.drive(direction * speed, -heading_error * GYRO_KP)

    robot.brake()

def turn_to_heading(target_deg, tolerance=1):
    delta = target_deg - hub.imu.heading()
    # Normalize to [-180, 180]
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    robot.turn(delta)
    # Fine-tune with proportional correction
    while abs(hub.imu.heading() - target_deg) > tolerance:
        error = target_deg - hub.imu.heading()
        robot.drive(0, error * GYRO_KP)
    robot.stop()

# ── Missions ──────────────────────────────────────────────────────────────────
def silo():
    # rotate 90 degrees about right wheel

    # smack front arm down to hit silo 3 times
    pass

def heavy_lifting():

    # turn left 20(?) degrees

    # move forward 300(?) mm

    # turn right 20 degrees + 45 degrees

    # forward approach to white and black line

    # pick up heavy box with front arm
    pass

def forge():
    # reverse 320 mm to line up with forge

    # turn left 90 degrees to knock open forge
    pass

def who_lived_here():
    # rotate left >90 degrees to flip house
    pass

def whats_on_sale():
    # rotate right to release house

    # back up a little

    # turn right ~45 degrees to face the line ish

    # drive forward to realign with black and white line

    # back up a little

    # turn right 45 degrees

    # reverse to position in front of raising roof

    # turn left 125 degrees to face rear toward roof 

    # push up the roof

    # back up a little

    # turn left 45 degrees to face the rear of the bot towards the line

    # drive backward a good bit

    # IF AFTER SECOND PART:
        # rotate 180 degrees to face the line

        # drive forward and realign with line

        # back up into the market

        # drive backward a little

        # turn left to face home

        # drive home

    # ELSE:

        # turn right to face ass to home

        # back it on up to base
    pass


# ── Main run ──────────────────────────────────────────────────────────────────
reset_odometry()
silo()
heavy_lifting()
forge()
who_lived_here()
whats_on_sale()
