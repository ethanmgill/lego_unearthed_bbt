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
FRONT_ARM_GEAR_RATIO = [[28,20]]  # output degrees per motor degree
REAR_ARM_GEAR_RATIO  = [[12,36], [12,20]]
REAR_ARM_STARTING_ANGLE = 155
FRONT_ARM_STARTING_ANGLE = -8

# ── Hardware init ─────────────────────────────────────────────────────────────
hub = PrimeHub()
left_motor  = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A) # reversed gearing for correct turn direction
front_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, gears=FRONT_ARM_GEAR_RATIO, reset_angle=False, profile=0) # arms
rear_motor  = Motor(Port.D, Direction.COUNTERCLOCKWISE, gears=REAR_ARM_GEAR_RATIO, reset_angle=False) # whacker
left_light_sensor = ColorSensor(Port.B)
right_light_sensor = ColorSensor(Port.F)
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

def turn_to_heading(target_deg, tolerance=3):
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

def turn_by(degrees, tolerance=3):
    """Gyro-corrected relative turn: positive = right (CW), negative = left (CCW)."""
    turn_to_heading(hub.imu.heading() + degrees, tolerance)

# ── Pivot helpers (one wheel stationary) ─────────────────────────────────────
# Motor rotation for a pivot = AXLE_TRACK * angle_deg * 2 / WHEEL_DIAMETER
# (derived from arc_length = AXLE_TRACK * angle_rad, then arc / circumference * 360)

def pivot_about_right_wheel(heading, speed=200):
    """Pivot with right wheel fixed to an absolute IMU heading (degrees CW from start)."""
    delta = heading - hub.imu.heading()
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    motor_deg = round(AXLE_TRACK_MM * abs(delta) * 2 / WHEEL_DIAMETER_MM)
    direction = 1 if delta > 0 else -1
    right_motor.hold()
    left_motor.run_angle(speed, direction * motor_deg)
    robot.reset()  # resync DriveBase after direct motor control

def pivot_about_left_wheel(heading, speed=200):
    """Pivot with left wheel fixed to an absolute IMU heading (degrees CW from start)."""
    delta = heading - hub.imu.heading()
    print(f"delta: {delta}")
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    motor_deg = round(AXLE_TRACK_MM * abs(delta) * 2 / WHEEL_DIAMETER_MM)
    direction = 1 if delta < 0 else -1
    left_motor.hold()
    right_motor.run_angle(speed, direction * motor_deg)
    robot.reset()

async def startup():
    await front_motor.run_until_stalled(-300, then=Stop.HOLD) # lower arm until stall to reset angle
    wait(200)
    reset_odometry()
    rear_motor.reset_angle(REAR_ARM_STARTING_ANGLE)
    front_motor.reset_angle(FRONT_ARM_STARTING_ANGLE)

# ── Missions ──────────────────────────────────────────────────────────────────
async def silo():
    # 260 mm
    # lift arm until stall
    await front_motor.run_until_stalled(300, then=Stop.HOLD) 
    # drive forward away from wall
    drive_straight(200)
    # drive out to line up with silo
    turn_by(-90, tolerance=5)
    drive_straight(70)
    turn_to_heading(0, tolerance=5)
    # approach silo
    #drive_straight(55)
    wait(200)
    # smack silo 3 times
    for i in range(1):
        turn_to_heading(3, tolerance=2)
        print(f"pushing down: #{i+1}")
        await front_motor.run_until_stalled(-500, then=Stop.HOLD) # push down on silo until stall
        wait(200)
        print(f"pulling up: #{i+1}")
        await front_motor.run_target(300, 60, then=Stop.COAST)
        await front_motor.run_until_stalled(300, then=Stop.HOLD)  # lift arm until stall
        wait(200)
    wait(200)


async def heavy_lifting():
    # transition from silo to heavy listing
    turn_to_heading(-45)
    drive_straight(200)
    turn_to_heading(0)
    # drive forward to line up with heavy object
    drive_straight(305, speed=200)
    # turn to face heavy object
    turn_to_heading(45, tolerance=2)
    # approach heavy object partially
    drive_straight(30, speed=100)
    # lower arm (Ethan)
    # await front_motor.run_target(50, 0, then=Stop.BRAKE)
    # front_motor.run_target(50, 0, then=Stop.HOLD)
    # lower arm (Raf)
    await front_motor.run_target(50, 10, then=Stop.BRAKE)
    front_motor.run_target(50, 10, then=Stop.HOLD)
    wait(200)

    # move forward to reach under object
    drive_straight(45) # commented out while testing raf's arm control
    # pickup object
    await front_motor.run_target(300, 100, then=Stop.HOLD)
    await front_motor.run_until_stalled(300, then=Stop.HOLD)  # lift arm until stall

    # await front_motor.run_until_stalled(300, then=Stop.HOLD)
    wait(200)


def forge():
    # back away from heavy object
    drive_straight(-90)
    # turn to set up for forge
    turn_to_heading(90)

    print('heading to forge pos..')
    while right_light_sensor.color() != Color.BLACK:
        left_motor.run_time(300, 10, wait=False)
        right_motor.run_time(300, 10, wait=False)
        wait(10)
        
    print('reached forge pos..')
    pivot_about_left_wheel(180)

    # OLD IMPLEMENTATION
    # # drive forward to line up with forge position
    # drive_straight(130)
    # # turn to face forge
    # pivot_about_left_wheel(45)
    # # drive backwards through forge, using appendage to push over lever
    # drive_straight(-180)


def who_lived_here():
    # turn about left wheel to line up with who lived here
    pivot_about_left_wheel(45)
    # drive backward to push house over
    drive_straight(-150)
    # drive forward, away from house
    drive_straight(50)


def whats_on_sale(return_via_market=False):
    turn_by(20)            # TUNE: degrees to release house
    drive_straight(-80)
    turn_by(45)
    drive_straight(300)    # TUNE: forward distance to realign with line
    drive_straight(-50)
    turn_by(45)
    drive_straight(-250)   # TUNE: reverse to position in front of roof
    turn_by(-125)
    rear_motor.run_angle(400, 120)   # push roof up
    wait(300)
    rear_motor.run_angle(300, -120)  # retract whacker
    drive_straight(-60)
    turn_by(-45)
    drive_straight(-400)   # TUNE: drive backward toward line

    if return_via_market:
        turn_by(180)
        drive_straight(200)    # TUNE: forward to realign with line
        drive_straight(-150)   # TUNE: back into market
        drive_straight(-50)
        turn_by(-90)
        drive_straight(600)    # TUNE: drive home
    else:
        turn_by(90)
        drive_straight(-500)   # TUNE: reverse to base


# ── Main run ──────────────────────────────────────────────────────────────────
async def main():

    await startup()

    await silo()
    await heavy_lifting()
    await forge()

    robot.stop()


    # TEST CODE:
    # for i in range(10):
    #     await rear_motor.run_target(target_angle=0+i*10, speed=100, wait=True)
    # print("position after whacker test:", rear_motor.angle())
    # await rear_motor.run_target(target_angle=0, speed=100, wait=True)
    
    # for i in range(-1, 8, 1):
    #     await front_motor.run_target(target_angle=0+i*10, speed=10, wait=True, then=Stop.HOLD)
    # await wait(2000)
    # print("position after arm test:", front_motor.angle())
    # front_motor.run_target(target_angle=0, speed=10, wait=True, then=Stop.HOLD)
    # print("position after arm reset:", front_motor.angle())
    # exit()
    # await wait(100000)

    #await silo()
    #await heavy_lifting()
    #await forge()
    #await who_lived_here()
    #await whats_on_sale()

run_task(main())
