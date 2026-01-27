package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that orients the robot to face a specific direction on the field.
 * Direction is specified in degrees (0° = right, 90° = up, 180° = left, -90° = down).
 * Runs until cancelled (e.g., button released).
 */
public class OrientInDirectionCommand extends OrientationCommandBase {
    private final double targetAngleDegrees;
    
    public OrientInDirectionCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator,
            double targetAngleDegrees) {
        super(drive, poseEstimator);
        this.targetAngleDegrees = targetAngleDegrees;
    }
    
    @Override
    protected double getTargetAngle() {
        return targetAngleDegrees;
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
