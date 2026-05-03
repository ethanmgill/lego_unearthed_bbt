# kanyes_new_workout_plan — Design Document

## File Structure

```
missions/
└── kanyes_new_workout_plan.py   ← single file running all 5 missions in sequence
```

---

## Implementation Plan

### Imports
| Module | Purpose |
|--------|---------|
| `pybricks.hubs.PrimeHub` | Hub access (IMU, speaker, light) |
| `pybricks.pupdevices.Motor` | Drive motor control |
| `pybricks.robotics.DriveBase` | Differential drive + odometry |
| `pybricks.parameters.Port, Direction, Stop` | Hardware references |
| `pybricks.tools.wait` | Timing |

### Configuration Constants
| Constant | Description |
|----------|-------------|
| `WHEEL_DIAMETER_MM` | Measured wheel diameter |
| `AXLE_TRACK_MM` | Center-to-center axle distance |
| `STRAIGHT_SPEED` / `STRAIGHT_ACCEL` | Linear movement speed and acceleration |
| `TURN_RATE` / `TURN_ACCEL` | Rotational speed and acceleration |
| `GYRO_KP` | Proportional gain for gyroscopic heading correction |

### Hardware Init
- `hub = PrimeHub()`
- `left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)`
- `right_motor = Motor(Port.A)`
- `robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)`

### Odometry Helpers
- **`reset_odometry()`** — calls `robot.reset()` to zero distance and angle tracking
- **`get_position()`** — returns `robot.state()` → `(distance_mm, heading_deg)`

### Gyro-Corrected Movement
- **`drive_straight(distance_mm, speed)`** — polls `hub.imu.heading()` each iteration and feeds `-error * GYRO_KP` into `robot.drive()` for continuous heading correction
- **`turn_to_heading(target_deg, tolerance=1)`** — computes normalized delta, calls `robot.turn(delta)`, then fine-tunes with a proportional loop until within tolerance

### Missions
| Function | Status |
|----------|--------|
| `whats_on_sale()` | In progress |
| `who_lived_here()` | Stub |
| `forge()` | Stub |
| `heavy_lifting()` | Stub |
| `silo()` | Stub |

### Main Run
```python
reset_odometry()
whats_on_sale()
who_lived_here()
forge()
heavy_lifting()
silo()
```

---

## Documentation

### Live-reading motor positions
SPIKE Prime motors have built-in rotational encoders. Use this monitor script — output streams to the VS Code debug console while the hub is connected:

```python
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

hub = PrimeHub()
motor_e = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_a = Motor(Port.A)

while True:
    print("E:", motor_e.angle(), "  A:", motor_a.angle(), "  heading:", hub.imu.heading())
    wait(100)
```

| Method | Returns |
|--------|---------|
| `motor.angle()` | Accumulated degrees since last `reset_angle()` |
| `motor.speed()` | Current speed in deg/s |
| `hub.imu.heading()` | Gyro heading in degrees |
| `robot.state()` | `(distance_mm, angle_deg)` from odometry |

> **Tip:** Call `motor.reset_angle(0)` to zero a motor's position before a run, the same way `robot.reset()` zeros the DriveBase.
