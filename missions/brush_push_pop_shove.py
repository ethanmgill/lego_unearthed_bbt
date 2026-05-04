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
REAR_ARM_STARTING_ANGLE = 145
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
        left_motor.hold()
        right_motor.hold()
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
async def surface_brushing():
    # prime front arm to be ready to pickup artifact
    front_motor.run_target(200, 15, then=Stop.HOLD)
    # drive forward to push brush
    drive_straight(740, speed=200)
    # pickup artifact
    await front_motor.run_target(200, 80, then=Stop.HOLD)
    # start lowering shove bar to prep for mineshaft
    rear_motor.run_target(300, 0, then=Stop.HOLD)
    while (not front_motor.done()):
        wait(10)
    # backup over brush
    drive_straight(-100, speed=400, then=Stop.HOLD)
    # # drive through brush again
    # drive_straight(150, speed=200)
    # # back away a little
    # drive_straight(-50, speed=150)
    # turn to face butt towards mine
    turn_to_heading(-115, tolerance=3) 
    # drive backward towards mine while lowering shove bar
    wait(450)
    drive_straight(-170, speed=200, then=Stop.HOLD)

    # ensure bar is down
    while (not rear_motor.done()):
        wait(10)
    # final approach to mine
    drive_straight(-60, speed=100, then=Stop.HOLD)
    # lift mine to send away cart
    await rear_motor.run_target(150, REAR_ARM_STARTING_ANGLE, then=Stop.HOLD)
    # lower arm and back away
    rear_motor.run_target(300, 0, then=Stop.HOLD)
    drive_straight(220, speed=300, then=Stop.HOLD)
    # pickup arm, turn to home, drive home
    rear_motor.run_target(300, REAR_ARM_STARTING_ANGLE, then=Stop.HOLD)
    turn_to_heading(-180, tolerance=5)
    drive_straight(450, speed=400, then=Stop.HOLD)

async def mission_2():
    pass

async def mission_3():
    pass

# ── Main run ──────────────────────────────────────────────────────────────────
async def main():
    await startup()

    await surface_brushing()
    await mission_2()
    await mission_3()

    robot.stop()

run_task(main())
