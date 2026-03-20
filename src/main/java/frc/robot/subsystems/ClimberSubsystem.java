package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.ClimbConstatns.*;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private final CANcoder climbEncoder = new CANcoder(15);

  private double offset = 0;

  private static final double CLIMBER_READY_DEGREES = 105.0;
  private static final double CLIMBER_CLIMB_DEGREES = -15;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public ClimberSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);

    // create the configuration for the climb moter, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    offset = getPosition();
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    climberMotor.set(power);
  }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  private double getPosition() {
    double degrees = climbEncoder.getPosition().getValue().in(Degrees) * (12.0 / 28.0);
    degrees -= offset;
    return degrees;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Encoder", getPosition());
    SmartDashboard.putNumber("Climber/Offset", offset);
  }

  public Command MoveToReady() {
    return Commands.runEnd(() -> {
      climberMotor.set(CLIMBER_MOTOR_DOWN_PERCENT);
    }, climberMotor::stopMotor, this).until(() -> getPosition() > CLIMBER_READY_DEGREES);
  }

  public Command Climb() {
    return Commands.runEnd(() -> {
      climberMotor.set(CLIMBER_MOTOR_UP_PERCENT);
    }, climberMotor::stopMotor, this).until(() -> getPosition() < CLIMBER_CLIMB_DEGREES);
  }

  public Command Reset() {
    return Commands.runEnd(() -> {
      climberMotor.set(CLIMBER_MOTOR_DOWN_PERCENT * -Math.signum(getPosition()));
    }, climberMotor::stopMotor, this).until(() -> Math.abs(getPosition()) < 5);
  }
}
