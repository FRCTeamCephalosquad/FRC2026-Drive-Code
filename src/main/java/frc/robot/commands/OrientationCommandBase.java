package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Base class for all orientation-based commands.
 * Handles rotation PID control, telemetry, and arcade drive output.
 * Subclasses need only specify target angle and optional forward speed.
 */
public abstract class OrientationCommandBase extends Command {
    protected final DriveSubsystem drive;
    protected final Supplier<Pose2d> poseEstimator;
    protected final PIDController rotationController;
    
    // Rotation PID constants - tune these for your robot
    protected static final double ROTATION_KP = 0.05;
    protected static final double ROTATION_KI = 0.0;
    protected static final double ROTATION_KD = 0.005;
    protected static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    protected static final double MAX_ROTATION_SPEED = 0.5;
    protected static final double MIN_ROTATION_SPEED = 0.02; // Deadband
    
    protected OrientationCommandBase(DriveSubsystem drive, Supplier<Pose2d> poseEstimator) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        // Set up rotation PID controller
        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(ANGLE_TOLERANCE_DEGREES);
        
        addRequirements(drive);
    }
    
    /**
     * Subclasses must implement this to provide the target angle in degrees.
     * @return Target angle in degrees (field-relative)
     */
    protected abstract double getTargetAngle();
    
    /**
     * Subclasses can override this to provide forward/backward motion.
     * @return Forward speed [-1.0, 1.0], default is 0 (rotation only)
     */
    protected double getForwardSpeed() {
        return 0;
    }
    
    /**
     * Subclasses can override this to customize telemetry prefix.
     * @return Telemetry prefix for SmartDashboard keys
     */
    protected String getTelemetryPrefix() {
        return this.getClass().getSimpleName();
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = poseEstimator.get();
        double currentHeading = currentPose.getRotation().getDegrees();
        double targetAngle = getTargetAngle();
        
        // Calculate rotation speed using PID
        double rotationSpeed = rotationController.calculate(currentHeading, targetAngle);
        
        // Clamp rotation speed
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, 
                                 Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        
        // Apply deadband to prevent oscillation
        if (Math.abs(rotationSpeed) < MIN_ROTATION_SPEED) {
            rotationSpeed = 0;
        }
        
        // Get forward speed from subclass
        double forwardSpeed = getForwardSpeed();
        
        // Telemetry for tuning
        String prefix = getTelemetryPrefix();
        SmartDashboard.putNumber(prefix + "/TargetAngle", targetAngle);
        SmartDashboard.putNumber(prefix + "/CurrentHeading", currentHeading);
        SmartDashboard.putNumber(prefix + "/AngleError", targetAngle - currentHeading);
        SmartDashboard.putNumber(prefix + "/RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber(prefix + "/ForwardSpeed", forwardSpeed);
        SmartDashboard.putBoolean(prefix + "/AtSetpoint", rotationController.atSetpoint());
        
        // Drive with both rotation and forward motion
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
