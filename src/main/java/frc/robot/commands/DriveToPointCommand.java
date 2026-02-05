package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that drives the robot to a specific point on the field.
 * The robot continuously adjusts its heading to face the target while driving.
 * Finishes when the robot reaches the target (within tolerance).
 * 
 * Uses a "continuous pursuit" approach where both rotation and forward speed
 * are controlled simultaneously for smooth path following.
 */
public class DriveToPointCommand extends OrientationCommandBase {
    private final Translation2d targetPoint;
    private final PIDController distanceController;
    
    // Distance PID constants - tune these for your robot
    private static final double DISTANCE_KP = 0.5;
    private static final double DISTANCE_KI = 0.0;
    private static final double DISTANCE_KD = 0.0;
    private static final double ARRIVAL_TOLERANCE_METERS = 0.15;
    private static final double MAX_FORWARD_SPEED = 0.2;
    private static final double MIN_FORWARD_SPEED = 0.1; // Minimum speed to overcome friction
    
    // Slow down as we approach the target
    private static final double SLOWDOWN_DISTANCE_METERS = 1.0;
    private static final double SLOWDOWN_MIN_SPEED = 0.2;
    
    public DriveToPointCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator,
            Translation2d targetPoint) {
        super(drive, poseEstimator);
        this.targetPoint = targetPoint;
        
        // Set up distance PID controller
        distanceController = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);
        distanceController.setTolerance(ARRIVAL_TOLERANCE_METERS);
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
        
        // Use PID to calculate base forward speed
        // Target distance is 0 (we want to reach the point)
        double forwardSpeed = distanceController.calculate(currentDistance, 0);
        
        // Apply slowdown as we approach
        if (currentDistance < SLOWDOWN_DISTANCE_METERS) {
            double slowdownFactor = currentDistance / SLOWDOWN_DISTANCE_METERS;
            double maxSpeed = SLOWDOWN_MIN_SPEED + (MAX_FORWARD_SPEED - SLOWDOWN_MIN_SPEED) * slowdownFactor;
            forwardSpeed = Math.min(forwardSpeed, maxSpeed);
        } else {
            forwardSpeed = Math.min(forwardSpeed, MAX_FORWARD_SPEED);
        }
        
        // Ensure minimum speed to overcome friction (unless very close)
        if (currentDistance > ARRIVAL_TOLERANCE_METERS && Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
            forwardSpeed = Math.signum(forwardSpeed) * MIN_FORWARD_SPEED;
        }
        
        // Additional telemetry
        SmartDashboard.putNumber(getTelemetryPrefix() + "/CurrentDistance", currentDistance);
        SmartDashboard.putNumber(getTelemetryPrefix() + "/TargetDistance", 0.0);
        SmartDashboard.putBoolean(getTelemetryPrefix() + "/HasArrived", isFinished());
        
        return -forwardSpeed;
    }
    
    @Override
    public boolean isFinished() {
        Pose2d currentPose = poseEstimator.get();
        double currentDistance = currentPose.getTranslation().getDistance(targetPoint);
        return currentDistance <= ARRIVAL_TOLERANCE_METERS;
    }
}
