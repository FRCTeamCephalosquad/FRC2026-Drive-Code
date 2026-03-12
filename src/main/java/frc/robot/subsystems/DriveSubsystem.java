
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.PoseSubsystem.EncoderIO;
import frc.robot.subsystems.PoseSubsystem.GyroIO;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;

@SuppressWarnings("removal")
public class DriveSubsystem extends SubsystemBase implements EncoderIO, GyroIO {

  // create brushed motors for drive
  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  // set up differential drive class
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);;

  // drivetrain constants
  private final double DRIVE_WIDTH = 0.55; // Meters
  private final double DRIVE_GEAR_RATIO = 8.46; // 8.46:1 Toughbox Mini S
  private final double WHEEL_DIAMETER = Inches.of(6).in(Meters);
  private final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;
  private final double ENCODER_RATIO = 1.0 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFRENCE;

  // Drive Info
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Distance.ofBaseUnits(DRIVE_WIDTH, Meters)); // TODO Measure This

  private final AHRS gyro = new AHRS(NavXComType.kUSB1);

  public DriveSubsystem() {

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Right Position", getRightPosition());
    SmartDashboard.putNumber("Drive Left Position", getLeftPosition());
    SmartDashboard.putNumber("Drive Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Drive Left Velocity", getLeftVelocity());

    SmartDashboard.putNumber("Rotation 2d", getRotation2d().getDegrees());
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble(), false));
  }

  @Override
  public double getLeftPosition() {
    return leftLeader.getEncoder().getPosition() * ENCODER_RATIO;
  }

  @Override
  public double getRightPosition() {
    return rightLeader.getEncoder().getPosition() * ENCODER_RATIO;
  }

  @Override
  public double getLeftVelocity() {
    return (leftLeader.getEncoder().getVelocity() * ENCODER_RATIO) / 60.0;
  }

  @Override
  public double getRightVelocity() {
    return (rightLeader.getEncoder().getVelocity() * ENCODER_RATIO) / 60.0;
  }

  @Override
  public Rotation2d getRotation2d() {
    double rawYaw = gyro.getYaw();
    return new Rotation2d(Units.degreesToRadians(-rawYaw));
  }

  public DifferentialDriveKinematics getDifferentialDriveKinematics() {
    return m_kinematics;
  }

  public void arcadeDrive(double s, double r) {
    // Expo handled in joystick drive command
    drive.arcadeDrive(s, r, false);
  }

  public void tankDrive(double l, double r) {
    drive.tankDrive(l, r, false);
  }

  ///////////////////////// SYSID STUFF
  ///
  ///
  /// Declare with the specific Mut* types, initialize via Unit.mutable()
  private final MutVoltage m_leftVoltage = Volts.mutable(0);
  private final MutVoltage m_rightVoltage = Volts.mutable(0);
  private final MutDistance m_leftDist = Meters.mutable(0);
  private final MutDistance m_rightDist = Meters.mutable(0);
  private final MutLinearVelocity m_leftVel = MetersPerSecond.mutable(0);
  private final MutLinearVelocity m_rightVel = MetersPerSecond.mutable(0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(.4).per(Second), Volts.of(3), Seconds.of(10), null), // default config, or
                                                                                            // customize ramp
                                                                                            // rate/timeout
      new SysIdRoutine.Mechanism(
          (voltage) -> {
            leftLeader.setVoltage(voltage.in(Volts));
            rightLeader.setVoltage(voltage.in(Volts));
          },
          (log) -> {
            log.motor("drive-left")
                .voltage(m_leftVoltage.mut_replace(
                    leftLeader.getBusVoltage() * leftLeader.getAppliedOutput(), Volts))
                .linearPosition(m_leftDist.mut_replace(
                    getLeftPosition(), Meters))
                .linearVelocity(m_leftVel.mut_replace(
                    getLeftVelocity(), MetersPerSecond));

            log.motor("drive-right")
                .voltage(m_rightVoltage.mut_replace(
                    rightLeader.getBusVoltage() * rightLeader.getAppliedOutput(), Volts))
                .linearPosition(m_rightDist.mut_replace(
                    getRightPosition(), Meters))
                .linearVelocity(m_rightVel.mut_replace(
                    getRightVelocity(), MetersPerSecond));
          },
          this));

  public Command sysIdQuasistaticForward() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticBackward() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicBackward() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }
}