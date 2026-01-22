package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;

public class KrakenDriveSubsystem extends SubsystemBase implements DriveSubsystem {
    // Drive Motors
    private final SparkMax m_leftDrive = new SparkMax(2, MotorType.kBrushed);
    private final SparkMax m_rightDrive = new SparkMax(3, MotorType.kBrushed);
    private final SparkMax m_leftDriveFollower = new SparkMax(4, MotorType.kBrushed);
    private final SparkMax m_rightDriveFollower = new SparkMax(5, MotorType.kBrushed);

    // Drive Control
    private final DifferentialDrive m_robotDrive;

    // Drive Info
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
            Distance.ofBaseUnits(.5, Meters)); // TODO Measure This

    // Encoders
    private final Encoder leftEncoder = new Encoder(0, 1);
    private final Encoder rightEncoder = new Encoder(2, 3);

    // Gyro
    private final AHRS gyro = new AHRS(NavXComType.kUSB1);

    @SuppressWarnings("removal")
    public KrakenDriveSubsystem() {

        final double NOMINAL_VOLTAGE = 12.0;

        // Right Motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.voltageCompensation(NOMINAL_VOLTAGE);
        rightConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        m_rightDrive.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set up right follower
        SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
        rightFollowConfig.follow(m_rightDrive);
        rightFollowConfig.voltageCompensation(NOMINAL_VOLTAGE);
        rightFollowConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // left Motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.voltageCompensation(NOMINAL_VOLTAGE);
        leftConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        leftConfig.inverted(true);
        m_leftDrive.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set up left follower
        SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        leftFollowConfig.follow(m_leftDrive);
        leftFollowConfig.voltageCompensation(NOMINAL_VOLTAGE);
        leftFollowConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        m_leftDriveFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        {
            // Encoder Setup
            final double wheelRadiusM = Inches.of(6).in(Meters);
            final double wheelCircumfrenceM = wheelRadiusM * Math.PI;
            final int ppr = 128;
            final double metersPerPulse = wheelCircumfrenceM / ppr;
            leftEncoder.setDistancePerPulse(metersPerPulse);
            rightEncoder.setDistancePerPulse(metersPerPulse);
        }

        m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
    }

    public void resetDistanceForward() {
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public double getDistanceForward() {
        return (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;
    }

    public void setMaxOutput(double m) {
        m_robotDrive.setMaxOutput(m);
    }

    public void stopMotor() {
        m_robotDrive.stopMotor();
    }

    // Command factory to create command to drive the robot with joystick inputs.
    public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        //TODO consider separate joystick version with deadband and expo!
        return this.run(
                () -> m_robotDrive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
    }

    @Override
    public DifferentialDriveKinematics getDifferentialDriveKinematics() {
        return m_kinematics;
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(-gyro.getYaw()));
    }

    @Override
    public double getLeftPosition() {
        return leftEncoder.getDistance();
    }

    @Override
    public double getRightPosition() {
        return rightEncoder.getDistance();
    }

    @Override
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    @Override
    public double getRightVelocity() {
        return rightEncoder.getRate();
    }
}
