from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.tools import wait, run_task

# ── Robot configuration ───────────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 88    # your wheels
AXLE_TRACK_MM = 145       # axle center-to-center distance
STRAIGHT_SPEED = 300      # mm/s
STRAIGHT_ACCEL = 600      # mm/s²
TURN_RATE = 150           # deg/s
TURN_ACCEL = 300          # deg/s²
GYRO_KP = 1.2             # proportional gain for heading correction
FRONT_ARM_GEAR_RATIO = [[28,20]]
REAR_ARM_GEAR_RATIO  = [[12,36], [12,20]]
REAR_ARM_STARTING_ANGLE = 155
FRONT_ARM_STARTING_ANGLE = -8

# ── Hardware init ─────────────────────────────────────────────────────────────
hub = PrimeHub()
left_motor  = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
front_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, gears=FRONT_ARM_GEAR_RATIO, reset_angle=False, profile=0)
rear_motor  = Motor(Port.D, Direction.COUNTERCLOCKWISE, gears=REAR_ARM_GEAR_RATIO, reset_angle=False)
left_light_sensor = ColorSensor(Port.B)
right_light_sensor = ColorSensor(Port.F)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
robot.settings(STRAIGHT_SPEED, STRAIGHT_ACCEL, TURN_RATE, TURN_ACCEL)

# ── Odometry helpers ──────────────────────────────────────────────────────────
def reset_odometry():
    robot.reset()

def get_position():
    """Returns (distance_mm, heading_deg) from last reset."""
    return robot.state()

# ── Gyro-corrected movement ───────────────────────────────────────────────────
def drive_straight(distance_mm, speed=STRAIGHT_SPEED, then=Stop.BRAKE):
    robot.reset()
    target_heading = hub.imu.heading()
    direction = 1 if distance_mm >= 0 else -1
    target_distance = abs(distance_mm)

    while abs(robot.distance()) < target_distance:
        heading_error = hub.imu.heading() - target_heading
        robot.drive(direction * speed, -heading_error * GYRO_KP)

    if then == Stop.BRAKE:
        robot.stop()
    elif then == Stop.HOLD:
        robot.hold()
    else: # COAST
        pass

def turn_to_heading(target_deg, tolerance=3):
    delta = target_deg - hub.imu.heading()
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    robot.turn(delta)
    while abs(hub.imu.heading() - target_deg) > tolerance:
        error = target_deg - hub.imu.heading()
        robot.drive(0, error * GYRO_KP)
    robot.stop()

def turn_by(degrees, tolerance=3):
    """Gyro-corrected relative turn: positive = right (CW), negative = left (CCW)."""
    turn_to_heading(hub.imu.heading() + degrees, tolerance)

# ── Pivot helpers (one wheel stationary) ─────────────────────────────────────
async def pivot_about_right_wheel(degrees, speed=200):
    """Pivot with right wheel fixed; positive = CW (heading increases)."""
    motor_deg = round(AXLE_TRACK_MM * abs(degrees) * 2 / WHEEL_DIAMETER_MM)
    direction = 1 if degrees > 0 else -1
    robot.stop()
    right_motor.hold()
    wait(50)
    await left_motor.run_angle(speed, direction * motor_deg)
    robot.reset()

async def pivot_about_left_wheel(degrees, speed=200):
    """Pivot with left wheel fixed; positive = CW (heading increases)."""
    motor_deg = round(AXLE_TRACK_MM * abs(degrees) * 2 / WHEEL_DIAMETER_MM)
    direction = 1 if degrees > 0 else -1
    robot.stop()
    left_motor.hold()
    wait(50)
    await right_motor.run_angle(speed, direction * motor_deg)
    robot.reset()

async def startup():
    await front_motor.run_until_stalled(-300, then=Stop.HOLD)
    wait(200)
    reset_odometry()
    rear_motor.reset_angle(REAR_ARM_STARTING_ANGLE)
    front_motor.reset_angle(FRONT_ARM_STARTING_ANGLE)

# ── Missions ──────────────────────────────────────────────────────────────────
async def mission_1():
    pass

async def mission_2():
    pass

async def mission_3():
    pass

# ── Main run ──────────────────────────────────────────────────────────────────
async def main():
    await startup()

    await mission_1()
    await mission_2()
    await mission_3()

    robot.stop()

run_task(main())
