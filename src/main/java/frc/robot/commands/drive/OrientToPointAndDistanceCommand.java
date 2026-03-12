package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that orients the robot to face a specific point while maintaining
 * a target distance from that point. The robot will drive forward or backward
 * to maintain the distance while continuously facing the point.
 * 
 * If the robot gets knocked around, it will reorient itself to the point
 * and adjust distance as needed.
 */
public abstract class OrientToPointAndDistanceCommand extends OrientationCommandBase {

    private final PIDController distanceController;

    // Distance PID constants - tune these for your robot
    private static final double DISTANCE_KP = 0.5;
    private static final double DISTANCE_KI = 0.0;
    private static final double DISTANCE_KD = 0.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.1;
    private static final double MAX_FORWARD_SPEED = 0.2;
    private static final double MIN_FORWARD_SPEED = 0.02; // Deadband

    // Angle threshold to prioritize orientation over distance correction
    private static final double LARGE_ANGLE_ERROR_THRESHOLD = 15.0; // degrees

    public OrientToPointAndDistanceCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator) {
        super(drive, poseEstimator);


        SmartDashboard.putNumber("Distance kP", DISTANCE_KP);
        SmartDashboard.putNumber("Distance kI", DISTANCE_KI);
        SmartDashboard.putNumber("Distance kD", DISTANCE_KD);

        // Set up distance PID controller
        distanceController = new PIDController(0, 0, 0);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);
    }

    
    public abstract Translation2d getTarget();
    public abstract double getDistance();

    @Override
    public void initialize() {
        super.initialize();
        double kP = SmartDashboard.getNumber("Distance kP", DISTANCE_KP);
        double kI = SmartDashboard.getNumber("Distance kI", DISTANCE_KI);
        double kD = SmartDashboard.getNumber("Distance kD", DISTANCE_KD);
        distanceController.setP(kP);
        distanceController.setI(kI);
        distanceController.setD(kD);
    }

    @Override
    protected double getTargetAngle() {
        Pose2d currentPose = poseEstimator.get();

        // Calculate angle to target point
        double deltaX = getTarget().getX() - currentPose.getX();
        double deltaY = getTarget().getY() - currentPose.getY();
        return Math.toDegrees(MathUtil.angleModulus(Math.atan2(deltaY, deltaX) - Math.PI));
    }

    @Override
    protected double getForwardSpeed() {
        Pose2d currentPose = poseEstimator.get();

        // Calculate current distance to target
        double currentDistance = currentPose.getTranslation().getDistance(getTarget());

        // Calculate angle error to decide if we should prioritize orientation
        double currentHeading = currentPose.getRotation().getDegrees();
        double targetAngle = getTargetAngle();
        double angleError = Math.abs(targetAngle - currentHeading);

        // Normalize angle error to [0, 180]
        if (angleError > 180) {
            angleError = 360 - angleError;
        }

        // If we're badly misaligned, don't drive forward/backward yet
        if (angleError > LARGE_ANGLE_ERROR_THRESHOLD) {
            SmartDashboard.putBoolean(getTelemetryPrefix() + "/PrioritizingOrientation", true);
            return 0;
        }

        SmartDashboard.putBoolean(getTelemetryPrefix() + "/PrioritizingOrientation", false);

        // Calculate forward/backward speed to maintain distance
        // Note: PID calculates (measurement - setpoint), so if we're too far,
        // it gives positive output (drive forward). If too close, negative (drive
        // backward).
        double forwardSpeed = distanceController.calculate(currentDistance, getDistance());

        // Clamp forward speed
        forwardSpeed = Math.max(-MAX_FORWARD_SPEED,
                Math.min(MAX_FORWARD_SPEED, forwardSpeed));

        // Apply deadband
        if (Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
            forwardSpeed = 0;
        }

        // Additional telemetry
        SmartDashboard.putNumber(getTelemetryPrefix() + "/CurrentDistance", currentDistance);
        SmartDashboard.putNumber(getTelemetryPrefix() + "/TargetDistance", getDistance());
        SmartDashboard.putNumber(getTelemetryPrefix() + "/DistanceError", currentDistance - getDistance());
        SmartDashboard.putBoolean(getTelemetryPrefix() + "/AtDistanceSetpoint", distanceController.atSetpoint());

        return forwardSpeed;
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
