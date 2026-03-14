package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.commands.drive.OrientInDirectionCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ClimbPosition extends SequentialCommandGroup{
    private static final Translation2d blue1 = new Translation2d(2, 3.25);
    private static final Translation2d blue2 = new Translation2d(1.3, 3.25 );

    public ClimbPosition(DriveSubsystem drive, Supplier<Pose2d> poseEstimator){
        addCommands(
            new DriveToPoint(drive, poseEstimator, this::getPoint1,0.2, 0.2),
            new OrientInDirectionCommand(drive, poseEstimator, 180).withTimeout(2),
            new DriveToPoint(drive, poseEstimator, this::getPoint2, 0.1, 0.05)
        );
    }

    private Translation2d getPoint1(){
        return blue1;
    }
    private Translation2d getPoint2(){
        return blue2;
    }
}
