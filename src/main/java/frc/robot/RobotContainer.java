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

import static frc.robot.Constants.FuelConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.OrientToPointCommand;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.KrakenDriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final KrakenDriveSubsystem driveSubsystem = new KrakenDriveSubsystem();
  private final CANFuelSubsystem ballSubsystem = null;// TODO new CANFuelSubsystem();

  private final PoseSubsystem poseSubsystem = new PoseSubsystem(driveSubsystem);

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
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
    poseSubsystem.setDefaultCommand(
        poseSubsystem.run(poseSubsystem::update));

    Translation2d blueTower = new Translation2d(0.01, 3.73);
    driverController.y().whileTrue(new OrientToPointCommand(driveSubsystem, poseSubsystem::getCurrentPose, blueTower));

    // While the left bumper on operator controller is held, intake Fuel
    if (ballSubsystem != null) {
      operatorController.leftBumper()
          .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
      // While the right bumper on the operator controller is held, spin up for 1
      // second, then launch fuel. When the button is released, stop.
      operatorController.rightBumper()
          .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
              .andThen(ballSubsystem.launchCommand())
              .finallyDo(() -> ballSubsystem.stop()));
      // While the A button is held on the operator controller, eject fuel back out
      // the intake
      operatorController.a()
          .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    }

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    driveSubsystem.setDefaultCommand(
        new JoystickDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX(),
            driveSubsystem));
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
