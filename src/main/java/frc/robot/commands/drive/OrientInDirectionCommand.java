package frc.robot.commands.drive;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that orients the robot to face a specific direction on the field.
 * Direction is specified in degrees (0° = right, 90° = up, 180° = left, -90° =
 * down).
 * Runs until cancelled (e.g., button released).
 */
public class OrientInDirectionCommand extends OrientationCommandBase {

    private final double blueAngle;

    private final Supplier<Double> forwardSpeedSupplier;

    public OrientInDirectionCommand(DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator, double blueAngle) {
                this(drive, poseEstimator, blueAngle, 0);
    }

    public OrientInDirectionCommand(DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator, double blueAngle, double speed) {
        super(drive, poseEstimator);
        this.blueAngle = blueAngle;
        forwardSpeedSupplier = () -> speed;
    }

    @Override
    protected double getForwardSpeed() {
        if (Math.abs(rotationController.getError()) < 10)
            return forwardSpeedSupplier.get();
        return 0;
    }

    @Override
    protected double getTargetAngle() {
        if (DriverStation.getAlliance().orElseThrow(
                () -> new NoSuchElementException("Alliance not available!")) == Alliance.Blue)
            return blueAngle;
        return Units.radiansToDegrees(MathUtil.angleModulus(Units.degreesToRadians(blueAngle + 180)));
    }

}
