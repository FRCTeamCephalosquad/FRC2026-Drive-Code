package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.balls.Launch;
import frc.robot.commands.balls.LaunchSequence;
import frc.robot.commands.balls.SpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;

public class ScootAndShoot extends SequentialCommandGroup {

    public static final double DRIVE_TIME = 1;
    public static final double DRIVE_SPEED = 0.2;
    public static final double AIM_TIME = 2;
    public static final double SHOOT_TIME = 5;

    static {
        SmartDashboard.putNumber("Auto/ScootAndShoot/DriveTime", DRIVE_TIME);
        SmartDashboard.putNumber("Auto/ScootAndShoot/DriveSpeed", DRIVE_SPEED);
        SmartDashboard.putNumber("Auto/ScootAndShoot/AimTime", AIM_TIME);
        SmartDashboard.putNumber("Auto/ScootAndShoot/ShootTime", SHOOT_TIME);
    }

    public ScootAndShoot(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem, Supplier<Pose2d> poseSupplier) {
        addCommands(
                
                Commands.runEnd(
                        () -> driveSubsystem
                                .arcadeDrive(SmartDashboard.getNumber("Auto/ScootAndShoot/DriveSpeed", DRIVE_SPEED), 0),
                        () -> driveSubsystem.arcadeDrive(0, 0),
                        driveSubsystem)
                        .withTimeout(SmartDashboard.getNumber("Auto/ScootAndShoot/DriveTime", DRIVE_TIME)),
                new AutoAimCommand(driveSubsystem, poseSupplier)
                        .withTimeout(SmartDashboard.getNumber("Auto/ScootAndShoot/AimTime", AIM_TIME)),
                Commands.sequence(
                        new SpinUp(ballSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                        new Launch(ballSubsystem)
                ).withTimeout(SmartDashboard.getNumber("Auto/ScootAndShoot/ShootTime", SHOOT_TIME)));
    }
}