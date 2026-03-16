// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.OperatorConstants.*;

import java.util.function.Supplier;

import frc.robot.autos.ScootAndShoot;
import frc.robot.autos.ScootShootAndClimb;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ClimbPosition;
import frc.robot.commands.balls.Eject;
import frc.robot.commands.balls.Intake;
import frc.robot.commands.balls.LaunchSequence;
import frc.robot.commands.drive.JoystickTankDriveCommand;
import frc.robot.commands.drive.OrientInDirectionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.PDUSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {

    // SUBSYSTEMS
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

    private final PoseSubsystem poseSubsystem = new PoseSubsystem(driveSubsystem);
    private final Supplier<Pose2d> getPose = poseSubsystem::getCurrentPose;

    @SuppressWarnings("unused") // Gets registered so periodic is called
    private final PDUSubsystem pduSubsystem = new PDUSubsystem();

    // CONTROLLERS
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        autoChooser.setDefaultOption("Shoot & Scoot",
                () -> new ScootAndShoot(driveSubsystem, fuelSubsystem, getPose));

        autoChooser.addOption("Scoot Shoot & Climb",
                () -> new ScootShootAndClimb(driveSubsystem, fuelSubsystem, climberSubsystem, getPose));

        SmartDashboard.putData("Auto choices", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

        // Fuesl and Climber both default to stop
        fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
        climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));

        // OPERATOR CONTROLS

        // Auto Aim
        operatorController.a().whileTrue(new AutoAimCommand(driveSubsystem,
                getPose));

        // Ball Handling
        operatorController.leftTrigger().whileTrue(new Intake(fuelSubsystem));
        operatorController.rightTrigger().whileTrue(new LaunchSequence(fuelSubsystem, hopperSubsystem));
        operatorController.start().whileTrue(new Eject(fuelSubsystem));

        // Hopper Control
        operatorController.povDown().whileTrue(
                Commands.sequence(
                        climberSubsystem.Reset(),
                        hopperSubsystem.lowerCommand()));
        operatorController.povUp().whileTrue(hopperSubsystem.raiseCommand());
        operatorController.y().whileTrue(hopperSubsystem.buttWiggle());

        // DRIVER CONTROLS

        // General Tank Drive
        driveSubsystem.setDefaultCommand(
                new JoystickTankDriveCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getRightY(),
                        driveSubsystem));

        // Field relative D-PAD helpers
        driverController.povUp()
                .whileTrue(new OrientInDirectionCommand(driveSubsystem, getPose, 0));
        driverController.povLeft()
                .whileTrue(new OrientInDirectionCommand(driveSubsystem, getPose, 90));
        driverController.povDown()
                .whileTrue(new OrientInDirectionCommand(driveSubsystem, getPose, 180));
        driverController.povRight()
                .whileTrue(new OrientInDirectionCommand(driveSubsystem, getPose, 270));

        // Climber Related
        driverController.a().whileTrue(
                Commands.sequence(
                        hopperSubsystem.raiseCommand(),
                        climberSubsystem.MoveToReady()));
        driverController.y().whileTrue(climberSubsystem.Climb());
        driverController.b().whileTrue(climberSubsystem.Reset());

        driverController.x().whileTrue(new ClimbPosition(driveSubsystem, getPose));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected().get();
    }
}
