package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.OrientToPointAndDistanceCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimCommand extends OrientToPointAndDistanceCommand {

    private static final Double AUTO_AIM_DISTANCE = 2.5;

    private static final Translation2d blueTower = new Translation2d(Inches.of(181.56).in(Meters),
            Inches.of(158.32).in(Meters));
    private static final Translation2d redTower = new Translation2d(Inches.of(470).in(Meters),
            Inches.of(158.32).in(Meters));

    public AutoAimCommand(DriveSubsystem drive,
            Supplier<Pose2d> poseEstimator) {
        super(drive, poseEstimator);
        SmartDashboard.putNumber("Auto Aim Distance", AUTO_AIM_DISTANCE);
    }

    @Override
    public Translation2d getTarget() {
        Alliance alliance = DriverStation.getAlliance().orElseThrow(
                () -> new NoSuchElementException("Alliance not available in Auto Aim!"));
        if (alliance == Alliance.Blue) {
            return blueTower;
        } else {
            return redTower;
        }
    }

    @Override
    public double getDistance() {
        return SmartDashboard.getNumber("Auto Aim Distance", AUTO_AIM_DISTANCE);
    }

}
