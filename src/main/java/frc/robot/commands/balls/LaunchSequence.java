
package frc.robot.commands.balls;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class LaunchSequence extends SequentialCommandGroup {

  public LaunchSequence(FuelSubsystem fuelSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        Commands.parallel(
            new Launch(fuelSubsystem),
            hopperSubsystem.buttWiggle()));
  }

}
