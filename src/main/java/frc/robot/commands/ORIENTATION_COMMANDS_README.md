# Orientation Command Family

A family of commands for controlling robot orientation and position using differential drive.

## Architecture

All commands inherit from `OrientationCommandBase`, which provides:
- Shared rotation PID controller
- Telemetry output to SmartDashboard
- Speed clamping and deadband
- Common arcade drive interface

Subclasses only need to implement:
- `getTargetAngle()` - What angle to face
- `getForwardSpeed()` - Optional forward/backward motion (default: 0)
- `isFinished()` - When to end the command

## Commands

### 1. OrientToPointCommand
**Purpose:** Points the robot at a specific field position  
**Finishes:** Never (runs until cancelled)  
**Usage:**
```java
new OrientToPointCommand(drive, poseEstimator, new Translation2d(5.0, 3.0))
```

### 2. OrientInDirectionCommand
**Purpose:** Points the robot in a specific field-relative direction  
**Finishes:** Never (runs until cancelled)  
**Usage:**
```java
// Face right (0°), up (90°), left (180°), down (-90°)
new OrientInDirectionCommand(drive, poseEstimator, 90.0)
```

### 3. OrientToPointAndDistanceCommand
**Purpose:** Faces a point while maintaining a specific distance from it  
**Behavior:** 
- Continuously orients toward the point
- Drives forward/backward to maintain distance
- If badly misaligned (>15°), prioritizes orientation before moving
- Self-corrects if knocked around

**Finishes:** Never (runs until cancelled)  
**Usage:**
```java
// Face the point at (5, 3) and stay 2 meters away
new OrientToPointAndDistanceCommand(drive, poseEstimator, new Translation2d(5.0, 3.0), 2.0)
```

### 4. DriveToPointCommand
**Purpose:** Drives to a specific point and stops when arrived  
**Behavior:**
- Continuously adjusts heading to face target
- Slows down as it approaches (within 1 meter)
- Uses continuous pursuit (smooth curves)

**Finishes:** Yes, when within 0.15 meters of target  
**Usage:**
```java
new DriveToPointCommand(drive, poseEstimator, new Translation2d(5.0, 3.0))
```

## Tuning Constants

### Rotation (OrientationCommandBase)
```java
ROTATION_KP = 0.05          // Increase for faster rotation response
ROTATION_KI = 0.0           // Usually keep at 0 for rotation
ROTATION_KD = 0.005         // Increase to reduce oscillation
ANGLE_TOLERANCE_DEGREES = 2.0
MAX_ROTATION_SPEED = 0.5    // Max rotation percentage
MIN_ROTATION_SPEED = 0.02   // Deadband to prevent tiny corrections
```

### Distance (OrientToPointAndDistanceCommand)
```java
DISTANCE_KP = 0.5           // Increase for faster distance correction
DISTANCE_KI = 0.0
DISTANCE_KD = 0.0
DISTANCE_TOLERANCE_METERS = 0.1
MAX_FORWARD_SPEED = 0.3
MIN_FORWARD_SPEED = 0.02
LARGE_ANGLE_ERROR_THRESHOLD = 15.0  // When to prioritize rotation over distance
```

### Distance (DriveToPointCommand)
```java
DISTANCE_KP = 0.5
ARRIVAL_TOLERANCE_METERS = 0.15     // How close is "arrived"
MAX_FORWARD_SPEED = 0.6
MIN_FORWARD_SPEED = 0.1             // Overcome static friction
SLOWDOWN_DISTANCE_METERS = 1.0      // Start slowing at 1m
SLOWDOWN_MIN_SPEED = 0.2            // Don't go slower than this when slowing
```

## SmartDashboard Telemetry

All commands output telemetry with their class name as prefix:

**Common (all commands):**
- `{CommandName}/TargetAngle`
- `{CommandName}/CurrentHeading`
- `{CommandName}/AngleError`
- `{CommandName}/RotationSpeed`
- `{CommandName}/ForwardSpeed`
- `{CommandName}/AtSetpoint`

**Distance commands additionally:**
- `{CommandName}/CurrentDistance`
- `{CommandName}/TargetDistance`
- `{CommandName}/DistanceError`
- `{CommandName}/AtDistanceSetpoint`

**OrientToPointAndDistanceCommand additionally:**
- `{CommandName}/PrioritizingOrientation` - True when angle error is too large

**DriveToPointCommand additionally:**
- `{CommandName}/HasArrived` - True when at target

## Example Robot.java Usage

```java
// In RobotContainer
private final DriveSubsystem drive = new DriveSubsystem();
private final Supplier<Pose2d> poseEstimator = drive::getPose;

// Bind to controller buttons
private void configureButtonBindings() {
    // Hold A to face a specific point
    controller.a().whileTrue(
        new OrientToPointCommand(drive, poseEstimator, new Translation2d(5.0, 3.0))
    );
    
    // Hold B to face forward on field
    controller.b().whileTrue(
        new OrientInDirectionCommand(drive, poseEstimator, 0.0)
    );
    
    // Hold X to maintain 2m from a point
    controller.x().whileTrue(
        new OrientToPointAndDistanceCommand(drive, poseEstimator, new Translation2d(5.0, 3.0), 2.0)
    );
    
    // Press Y to drive to a point (finishes automatically)
    controller.y().onTrue(
        new DriveToPointCommand(drive, poseEstimator, new Translation2d(5.0, 3.0))
    );
}
```

## Tuning Process

1. **Start with rotation tuning** (use OrientInDirectionCommand):
   - Watch `{Command}/AngleError` on SmartDashboard
   - Increase `ROTATION_KP` until it rotates quickly but overshoots slightly
   - Add `ROTATION_KD` to reduce overshoot
   - Adjust `MIN_ROTATION_SPEED` if it oscillates when close to target

2. **Tune distance control** (use OrientToPointAndDistanceCommand):
   - Set a target distance and watch `{Command}/DistanceError`
   - Increase `DISTANCE_KP` until it approaches quickly
   - Adjust `LARGE_ANGLE_ERROR_THRESHOLD` if it drives before properly oriented
   - Adjust `MIN_FORWARD_SPEED` if it doesn't move when close to setpoint

3. **Tune approach behavior** (use DriveToPointCommand):
   - Watch the robot approach a target
   - Adjust `SLOWDOWN_DISTANCE_METERS` and `SLOWDOWN_MIN_SPEED` for smooth deceleration
   - Adjust `ARRIVAL_TOLERANCE_METERS` based on how precise you need positioning

## Notes

- All commands require a properly configured pose estimator (odometry or vision)
- Commands assume field coordinates in meters
- Angles are in degrees, field-relative (0° = right, 90° = up, etc.)
- Differential drive can drive forward, backward, and rotate, but cannot strafe
