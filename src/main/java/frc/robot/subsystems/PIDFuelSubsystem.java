// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class PIDFuelSubsystem extends SubsystemBase {
  private final SparkMax LeftIntakeLauncher;
  private final SparkMax RightIntakeLauncher; // Leader
  private final SparkMax Indexer;

  private final SparkClosedLoopController launcherPID;
  private final RelativeEncoder launcherEncoder;

  private double targetRPM = 0.0;

  // Launcher PID gains
  // Tuning order: get KFF close first (1 / NEO free-speed RPM),
  // then nudge KP to eliminate steady-state error. KI and KD are
  // rarely needed for a flywheel — leave them at 0 until you have a reason.
  public static final double LAUNCHER_KP = 0.0001;
  public static final double LAUNCHER_KI = 0.0;
  public static final double LAUNCHER_KD = 0.0;
  public static final double LAUNCHER_KFF = 1.0 / 5676.0; // 1 / NEO free-speed RPM

  // Target launch speed and at-speed tolerance
  public static final double LAUNCHING_LAUNCHER_RPM = 4000.0; // tune to your shot distance
  public static final double LAUNCHER_RPM_TOLERANCE = 100.0; // ± RPM considered "at speed"

  @SuppressWarnings("removal")
  public PIDFuelSubsystem() {
    LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

    // --- Indexer config (unchanged) ---
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Right (leader) launcher config with velocity PID ---
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast);

    leaderConfig.closedLoop
        .p(LAUNCHER_KP)
        .i(LAUNCHER_KI)
        .d(LAUNCHER_KD)
        .velocityFF(LAUNCHER_KFF) // FF = 1 / NEO free-speed RPM is a good starting point
        .outputRange(-1, 1);

    RightIntakeLauncher.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Left (follower) launcher config ---
    // inverted=true so both wheels spin in the same "shoot" direction
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(RightIntakeLauncher, true);
    LeftIntakeLauncher.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Grab PID controller and encoder from the leader
    launcherPID = RightIntakeLauncher.getClosedLoopController();
    launcherEncoder = RightIntakeLauncher.getEncoder();

    // Dashboard tuning values
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
    SmartDashboard.putNumber("Launcher kP", LAUNCHER_KP);
    SmartDashboard.putNumber("Launcher kI", LAUNCHER_KI);
    SmartDashboard.putNumber("Launcher kD", LAUNCHER_KD);
    SmartDashboard.putNumber("Launcher kFF", LAUNCHER_KFF);
  }

  /**
   * Sets launcher wheel target velocity via closed-loop PID.
   * 
   * @param rpm Target RPM (positive = shooting direction)
   */
  @SuppressWarnings("removal")
  public void setLauncherRPM(double rpm) {
    targetRPM = rpm;
    if (rpm == 0) {
      RightIntakeLauncher.set(0); // Coast to stop instead of holding 0 RPM
    } else {
      launcherPID.setReference(rpm, ControlType.kVelocity);
    }
  }

  /** Returns current RPM of the leader motor's encoder. */
  public double getLauncherRPM() {
    return launcherEncoder.getVelocity();
  }

  /**
   * Returns true when the launcher is within tolerance of the target RPM.
   * Useful for "ready to shoot" checks in commands.
   */
  public boolean isAtTargetRPM() {
    return Math.abs(getLauncherRPM() - targetRPM) < LAUNCHER_RPM_TOLERANCE;
  }

  /**
   * Reads PID gains from SmartDashboard and re-applies them to the leader
   * controller.
   * Call this from a button binding after adjusting dashboard values.
   * Uses kNoPersistParameters to avoid flash writes during tuning —
   * once happy with the values, copy them into Constants.java.
   */
  @SuppressWarnings("removal")
  public void updatePIDGains() {
    double kP = SmartDashboard.getNumber("Launcher kP", LAUNCHER_KP);
    double kI = SmartDashboard.getNumber("Launcher kI", LAUNCHER_KI);
    double kD = SmartDashboard.getNumber("Launcher kD", LAUNCHER_KD);
    double kFF = SmartDashboard.getNumber("Launcher kFF", LAUNCHER_KFF);

    SparkMaxConfig updatedConfig = new SparkMaxConfig();
    updatedConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF);

    RightIntakeLauncher.configure(
        updatedConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setFeederRoller(double power) {
    Indexer.set(power);
  }

  public void stop() {
    Indexer.set(0);
    setLauncherRPM(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher RPM", getLauncherRPM());
    SmartDashboard.putNumber("Launcher Target RPM", targetRPM);
    SmartDashboard.putBoolean("Launcher At Speed", isAtTargetRPM());
  }
}