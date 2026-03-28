package frc.robot.commands;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.commands.drive.OrientInDirectionCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class ClimbPosition extends SequentialCommandGroup {
    private static final Translation2d blue1 = new Translation2d(2, 3.25);
    private static final Translation2d blue2 = new Translation2d(1.3, 3.25);
    private static final Translation2d red1 = new Translation2d(14.4, 4.78);
    private static final Translation2d red2 = new Translation2d(15.2, 4.78);

    public ClimbPosition(DriveSubsystem drive, ClimberSubsystem climb, HopperSubsystem hopper, Supplier<Pose2d> poseEstimator) {
        addCommands(    
                Commands.parallel(
                    Commands.sequence(
                        new DriveToPoint(drive, poseEstimator, this::getPoint1, 0.2, 0.2),
                        new OrientInDirectionCommand(drive, poseEstimator, 180).withTimeout(1)
                    ),
                    Commands.sequence(
                        hopper.raiseCommand(),
                        climb.MoveToReady())
                ),
               
                new DriveToPoint(drive, poseEstimator, this::getPoint2, 0.1, 0.05).withTimeout(3));
    }

    private Translation2d getPoint1() {
        Alliance alliance = DriverStation.getAlliance().orElseThrow(
                () -> new NoSuchElementException("Alliance not available in Auto Aim!"));
        if (alliance == Alliance.Blue) {
            return blue1;
        } else {
            return red1;
        }
    }

    private Translation2d getPoint2() {
        Alliance alliance = DriverStation.getAlliance().orElseThrow(
                () -> new NoSuchElementException("Alliance not available in Auto Aim!"));
        if (alliance == Alliance.Blue) {
            return blue2;
        } else {
            return red2;
        }
    }
}
