package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.OperatorConstants.*;

public class JoystickArcadeDriveCommand extends Command {
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier zRotationSupplier;
    private final DriveSubsystem driveSubsystem;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(DRIVE_FB_SLEW_LIMIT);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(DRIVE_ROTATE_SLEW_LIMIT);

    public JoystickArcadeDriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation, DriveSubsystem driveSubsystem) {
        this.xSpeedSupplier = xSpeed;
        this.zRotationSupplier = zRotation;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    /**
     * Scaling, Deadband and Expo are handled here, drive subsystem
     * deals in absolutes so it can accept input from PIDs etc
     */
    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.getAsDouble();
        double zRotation = zRotationSupplier.getAsDouble();

        // Clamp value
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        // Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, DRIVE_DEADBAND);
        zRotation = MathUtil.applyDeadband(zRotation, DRIVE_DEADBAND);

        // Apply Expo
        xSpeed = MathUtil.copyDirectionPow(xSpeed, DRIVE_EXPO);
        zRotation = MathUtil.copyDirectionPow(zRotation, DRIVE_EXPO);

        // Apply scaling
        xSpeed = xSpeed * DRIVE_SCALING;
        zRotation = zRotation * ROTATION_SCALING;

        // Limit slew rates
        xSpeed = xLimiter.calculate(xSpeed);
        zRotation = rotationLimiter.calculate(zRotation);

        // Pass into drive
        driveSubsystem.arcadeDrive(xSpeed, zRotation);
    }
}