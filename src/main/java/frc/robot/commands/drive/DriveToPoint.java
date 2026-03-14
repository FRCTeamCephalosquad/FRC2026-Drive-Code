package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveOrientationConstants.*;

public class DriveToPoint extends Command {

    private final DriveSubsystem m_drive;
    private final Pose2d m_target;
    Supplier<Pose2d> m_poseEstimator;

    private final PIDController m_turnPID   = new PIDController(0,0,0);
    private final PIDController m_drivePID  = new PIDController(0,0,0);

    private static final double DISTANCE_TOLERANCE = 0.1; // meters

    public DriveToPoint(DriveSubsystem drive, Pose2d target, Supplier<Pose2d> poseEstimator ) {
        m_drive  = drive;
        m_target = target;
        m_poseEstimator = poseEstimator;
        m_turnPID.enableContinuousInput(-180, 180);
        m_drivePID.setSetpoint(0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        super.initialize();
        double kPd = SmartDashboard.getNumber("Distance kP", DISTANCE_KP);
        double kId = SmartDashboard.getNumber("Distance kI", DISTANCE_KI);
        double kDd = SmartDashboard.getNumber("Distance kD", DISTANCE_KD);
        m_drivePID.setP(kPd);
        m_drivePID.setI(kId);
        m_drivePID.setD(kDd);

        double kPt = SmartDashboard.getNumber("Orient kP", ROTATION_KP);
        double kIt = SmartDashboard.getNumber("Orient kI", ROTATION_KI);
        double kDt = SmartDashboard.getNumber("Orient kD", ROTATION_KD);
        m_turnPID.setP(kPt);
        m_turnPID.setI(kIt);
        m_turnPID.setD(kDt);
    }

    @Override
    public void execute() {
        Pose2d current = m_poseEstimator.get();

        double dx = m_target.getX() - current.getX();
        double dy = m_target.getY() - current.getY();
        double distance = Math.hypot(dx, dy);

        // Desired heading to the point (degrees, field-relative)
        double desiredHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double currentHeadingDeg = current.getRotation().getDegrees();

        double turnOutput  = m_turnPID.calculate(currentHeadingDeg, desiredHeadingDeg);
        double driveOutput = m_drivePID.calculate(-distance); // negative because setpoint is 0

        // Clamp drive output so it doesn't go crazy
        driveOutput = Math.min(driveOutput, 0.5);

        m_drive.arcadeDrive(driveOutput, turnOutput);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = m_poseEstimator.get();
        double dx = m_target.getX() - current.getX();
        double dy = m_target.getY() - current.getY();
        return Math.hypot(dx, dy) < DISTANCE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }
}