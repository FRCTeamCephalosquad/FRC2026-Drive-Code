package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;

public interface DriveSubsystem extends PoseSubsystem.EncoderIO, PoseSubsystem.GyroIO {
    public DifferentialDriveKinematics getDifferentialDriveKinematics();

    // Command factory to create command to drive the robot with joystick inputs.
    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation);
}
