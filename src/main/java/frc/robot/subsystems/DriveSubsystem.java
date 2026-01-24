package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.PoseSubsystem.EncoderIO;
import frc.robot.subsystems.PoseSubsystem.GyroIO;

public interface DriveSubsystem extends EncoderIO, GyroIO, Subsystem {
    DifferentialDriveKinematics getDifferentialDriveKinematics();

    void arcadeDrive(double s, double r);
}
