// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.OrientToPointCommand;
import frc.robot.commands.balls.Eject;
import frc.robot.commands.balls.Intake;
import frc.robot.commands.balls.LaunchSequence;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PDUSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {

  // SUBSYSTEMS
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final PDUSubsystem pduSubsystem = new PDUSubsystem();
  private final PoseSubsystem poseSubsystem = new PoseSubsystem(driveSubsystem);

  // CONTROLLERS
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    // autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem,
    // ballSubsystem));

  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {

    // Normal mode for pose subsystem
    pduSubsystem.setDefaultCommand(pduSubsystem.run(pduSubsystem::updateCurrent));

    // Drive Default
    driveSubsystem.setDefaultCommand(
        new JoystickDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX(),
            driveSubsystem));

    // Fuesl and Climber both default to stop
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));

    // Test orientation code
    Translation2d blueTower = new Translation2d(0.01, 3.73);
    driverController.y().whileTrue(new OrientToPointCommand(driveSubsystem, poseSubsystem::getCurrentPose, blueTower));

    //Some probably stupid operator commands
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    operatorController.a().whileTrue(new Eject(fuelSubsystem));
    operatorController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    operatorController.povUp().whileTrue(new ClimbUp(climberSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
