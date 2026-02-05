package frc.robot.commands;

import java.util.function.Supplier;

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
public class OrientToPointAndDistanceCommand extends OrientationCommandBase {
    private final Translation2d targetPoint;
    private final double targetDistanceMeters;
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
            Supplier<Pose2d> poseEstimator,
            Translation2d targetPoint,
            double targetDistanceMeters) {
        super(drive, poseEstimator);
        this.targetPoint = targetPoint;
        this.targetDistanceMeters = targetDistanceMeters;
        
        // Set up distance PID controller
        distanceController = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);
    }
    
    @Override
    protected double getTargetAngle() {
        Pose2d currentPose = poseEstimator.get();
        
        // Calculate angle to target point
        double deltaX = targetPoint.getX() - currentPose.getX();
        double deltaY = targetPoint.getY() - currentPose.getY();
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }
    
    @Override
    protected double getForwardSpeed() {
        Pose2d currentPose = poseEstimator.get();
        
        // Calculate current distance to target
        double currentDistance = currentPose.getTranslation().getDistance(targetPoint);
        
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
        // it gives positive output (drive forward). If too close, negative (drive backward).
        double forwardSpeed = distanceController.calculate(currentDistance, targetDistanceMeters);
        
        // Clamp forward speed
        forwardSpeed = Math.max(-MAX_FORWARD_SPEED, 
                                Math.min(MAX_FORWARD_SPEED, forwardSpeed));
        
        // Apply deadband
        if (Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
            forwardSpeed = 0;
        }
        
        // Additional telemetry
        SmartDashboard.putNumber(getTelemetryPrefix() + "/CurrentDistance", currentDistance);
        SmartDashboard.putNumber(getTelemetryPrefix() + "/TargetDistance", targetDistanceMeters);
        SmartDashboard.putNumber(getTelemetryPrefix() + "/DistanceError", currentDistance - targetDistanceMeters);
        SmartDashboard.putBoolean(getTelemetryPrefix() + "/AtDistanceSetpoint", distanceController.atSetpoint());
        
        return -forwardSpeed;
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
