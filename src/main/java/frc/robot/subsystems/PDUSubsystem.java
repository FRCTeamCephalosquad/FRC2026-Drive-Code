package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDUSubsystem extends SubsystemBase {

    PowerDistribution pdu = new PowerDistribution(1, ModuleType.kRev);

    @Override
    public void periodic() {

        // Send power information
        SmartDashboard.putNumber("Voltage", pdu.getVoltage());
        SmartDashboard.putNumber("Total Current", pdu.getTotalCurrent());

        SmartDashboard.putNumber("Drive R1 Current", pdu.getCurrent(9));
        SmartDashboard.putNumber("Drive R2 Current", pdu.getCurrent(8));
        SmartDashboard.putNumber("Drive L1 Current", pdu.getCurrent(18));
        SmartDashboard.putNumber("Drive L2 Current", pdu.getCurrent(19));

        SmartDashboard.putNumber("Launcher Left", pdu.getCurrent(6));
        SmartDashboard.putNumber("Launcher Right", pdu.getCurrent(4));

        // These two might be swapped
        SmartDashboard.putNumber("Indexer", pdu.getCurrent(5));
        SmartDashboard.putNumber("Climber", pdu.getCurrent(7));

        SmartDashboard.putNumber("Hopper", pdu.getCurrent(3));
        SmartDashboard.putNumber("Vision Current", pdu.getCurrent(0));

        SmartDashboard.putNumber("RIO Current", pdu.getCurrent(20));
        SmartDashboard.putNumber("Radio Current", pdu.getCurrent(21));
        SmartDashboard.putNumber("Fan Current", pdu.getCurrent(22));

    }

}
