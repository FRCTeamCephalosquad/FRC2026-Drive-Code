package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that continuously orients the robot to face a specific point on the field.
 * Runs until cancelled (e.g., button released).
 */
public class OrientToPointCommand extends OrientationCommandBase {
    private final Translation2d targetPoint;
    
    public OrientToPointCommand(
            DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator,
            Translation2d targetPoint) {
        super(drive, poseEstimator);
        this.targetPoint = targetPoint;
    }
    
    @Override
    protected double getTargetAngle() {
        Pose2d currentPose = poseEstimator.get();
        
        // Calculate angle to target point
        double deltaX = targetPoint.getX() - currentPose.getX();
        double deltaY = targetPoint.getY() - currentPose.getY();
        return Math.toDegrees(MathUtil.angleModulus(Math.atan2(deltaY, deltaX)-Math.PI));
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
