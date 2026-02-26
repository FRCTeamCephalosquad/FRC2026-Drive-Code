package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.OperatorConstants.*;

public class JoystickTankDriveCommand extends Command {
    private final DoubleSupplier lSpeedSupplier;
    private final DoubleSupplier rSpeedSupplier;
    private final DriveSubsystem driveSubsystem;

    private final SlewRateLimiter rLimiter = new SlewRateLimiter(DRIVE_FB_SLEW_LIMIT);
    private final SlewRateLimiter lLimiter = new SlewRateLimiter(DRIVE_FB_SLEW_LIMIT);

    public JoystickTankDriveCommand(DoubleSupplier lSpeed, DoubleSupplier rSpeed, DriveSubsystem driveSubsystem) {
        this.lSpeedSupplier = lSpeed;
        this.rSpeedSupplier = rSpeed;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    /**
     * Scaling, Deadband and Expo are handled here, drive subsystem
     * deals in absolutes so it can accept input from PIDs etc
     */
    @Override
    public void execute() {
        double lSpeed = lSpeedSupplier.getAsDouble();
        double rSpeed = rSpeedSupplier.getAsDouble();

        // Clamp value
        lSpeed = MathUtil.clamp(lSpeed, -1.0, 1.0);
        rSpeed = MathUtil.clamp(rSpeed, -1.0, 1.0);

        // Apply deadband
        lSpeed = MathUtil.applyDeadband(lSpeed, DRIVE_DEADBAND);
        rSpeed = MathUtil.applyDeadband(rSpeed, DRIVE_DEADBAND);

        // apply expo
        lSpeed = MathUtil.copyDirectionPow(lSpeed, DRIVE_EXPO);
        rSpeed = MathUtil.copyDirectionPow(rSpeed, DRIVE_EXPO);

        // apply scaling
        lSpeed = lSpeed * DRIVE_SCALING;
        rSpeed = rSpeed * DRIVE_SCALING;

        // Limit slew rates
        lSpeed = lLimiter.calculate(lSpeed);
        rSpeed = rLimiter.calculate(rSpeed);

        // Pass into drive
        driveSubsystem.tankDrive(lSpeed, rSpeed);
    }
}