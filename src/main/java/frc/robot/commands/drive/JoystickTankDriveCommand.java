package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.OperatorConstants.*;

public class JoystickTankDriveCommand extends Command {

    public enum StraightLineMode {
        NONE,
        INPUT_SYNC,
        GYRO_HOLD
    }

    private static final double DRIVE_SYNC_THRESHOLD = 0.1;
    private static final double HEADING_KP = 0.01;
    private static final double MAX_HEADING_CORRECTION = 0.1;

    private final StraightLineMode straightLineMode = StraightLineMode.INPUT_SYNC;
    private boolean holdingHeading = false;
    private double headingTarget = 0.0;

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
        SmartDashboard.putBoolean("Holding Heading", false);
    }

    @Override
    public void initialize() {
        holdingHeading = false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.tankDrive(0, 0);
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

        // Straight-line assist
        double diff = Math.abs(lSpeed - rSpeed);
        boolean isStraight = diff < DRIVE_SYNC_THRESHOLD;

        switch (straightLineMode) {
            case INPUT_SYNC:
                if (isStraight) {
                    double avg = (lSpeed + rSpeed) / 2.0;
                    lSpeed = avg;
                    rSpeed = avg;
                }
                SmartDashboard.putBoolean("Holding Heading", isStraight);
                break;

            case GYRO_HOLD:
                if (isStraight) {
                    double avg = (lSpeed + rSpeed) / 2.0;
                    double direction = Math.signum(avg); // +1 forward, -1 backward, 0 stopped
                    double headingDegrees = driveSubsystem.getRotation2d().getDegrees();
                    if (!holdingHeading) {
                        headingTarget = headingDegrees;
                        holdingHeading = true;
                    }
                    double error = MathUtil.inputModulus(headingTarget - headingDegrees, -180, 180);
                    double correction = MathUtil.clamp(HEADING_KP * error,
                            -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
                    lSpeed = avg + correction * direction;
                    rSpeed = avg - correction * direction;
                } else {
                    holdingHeading = false;
                }
                SmartDashboard.putBoolean("Holding Heading", isStraight);
                break;

            case NONE:
            default:
                break;
        }

        driveSubsystem.tankDrive(lSpeed, rSpeed);
    }

}