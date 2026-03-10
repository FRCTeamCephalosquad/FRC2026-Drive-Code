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

    @SuppressWarnings("removal")
    public HopperSubsystem() {
        // create brushed motors for each of the motors on the launcher mechanism
        motor = new SparkMax(10, MotorType.kBrushless);

        // create the configuration for the climb moter, set a current limit and apply
        // the config to the controller
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(20);
        motorConfig.idleMode(IdleMode.kCoast);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hopper Position", motor.getEncoder().getPosition());
    }

    public Command lowerCommand(){
        return Commands.runEnd(()->{
            motor.set(.75);
        }, motor::stopMotor, this);
    }

    public Command raiseCommand(){
        return Commands.runEnd(()->{
            motor.set(-.75);
        }, motor::stopMotor, this);
    }

}
