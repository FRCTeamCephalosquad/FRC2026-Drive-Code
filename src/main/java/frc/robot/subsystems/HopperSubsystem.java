package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax motor;

    private static final double DOWN_LIMIT = 220;
    private static final double WIGGLE_LIMIT = 150;
    private static final double UP_LIMIT = 0;

    @SuppressWarnings("removal")
    public HopperSubsystem() {
        motor = new SparkMax(10, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(20);
        motorConfig.idleMode(IdleMode.kBrake);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Position", motor.getEncoder().getPosition());
    }

    public Command buttWiggle() {
        return Commands.sequence(
                lowerCommand(),
                raiseCommand().until(() -> motor.getEncoder().getPosition() < WIGGLE_LIMIT))
                .repeatedly()
                .finallyDo(interrupted -> lowerCommand().schedule());
    }

    public Command lowerCommand() {
        return Commands.runEnd(() -> {
            motor.set(.75);
        }, motor::stopMotor, this).until(() -> motor.getEncoder().getPosition() > DOWN_LIMIT);
    }

    public Command raiseCommand() {
        return Commands.runEnd(() -> {
            motor.set(-.75);
        }, motor::stopMotor, this).until(() -> motor.getEncoder().getPosition() < UP_LIMIT);
    }

}
