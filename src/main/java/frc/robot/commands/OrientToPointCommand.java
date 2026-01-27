package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class OrientToPointCommand extends Command {
    private final DriveSubsystem drive;
    private final Supplier<Pose2d> poseEstimator;
    private final Translation2d targetPoint;
    private final PIDController rotationController;
    
    // Tuning constants - adjust these for your robot
    private static final double KP = 0.05;
    private static final double KI = 0.0;
    private static final double KD = 0.005;
    private static final double TOLERANCE_DEGREES = 2.0;
    private static final double MAX_ROTATION_SPEED = 0.5; // Percentage
    private static final double MIN_ROTATION_SPEED = 0.02; // Deadband to prevent oscillation
    
    public OrientToPointCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator,
            Translation2d targetPoint) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.targetPoint = targetPoint;
        
        // Set up PID controller for rotation
        rotationController = new PIDController(KP, KI, KD);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(TOLERANCE_DEGREES);
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = poseEstimator.get();
        
        // Calculate angle to target
        double deltaX = targetPoint.getX() - currentPose.getX();
        double deltaY = targetPoint.getY() - currentPose.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        // Get current robot heading
        double currentHeading = currentPose.getRotation().getDegrees();
        
        // Calculate rotation speed using PID
        double rotationSpeed = rotationController.calculate(currentHeading, angleToTarget);
        
        // Clamp rotation speed
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, 
                                 Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        
        // Apply deadband to prevent oscillation
        if (Math.abs(rotationSpeed) < MIN_ROTATION_SPEED) {
            rotationSpeed = 0;
        }
        
        // Telemetry for tuning
        SmartDashboard.putNumber("OrientToPoint/AngleToTarget", angleToTarget);
        SmartDashboard.putNumber("OrientToPoint/CurrentHeading", currentHeading);
        SmartDashboard.putNumber("OrientToPoint/Error", angleToTarget - currentHeading);
        SmartDashboard.putNumber("OrientToPoint/RotationSpeed", rotationSpeed);
        SmartDashboard.putBoolean("OrientToPoint/AtSetpoint", rotationController.atSetpoint());
        
        // Drive with rotation only (no forward/backward movement)
        drive.arcadeDrive(0, rotationSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }
}