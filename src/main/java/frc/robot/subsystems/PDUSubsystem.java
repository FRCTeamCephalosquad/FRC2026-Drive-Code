package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDUSubsystem extends SubsystemBase {

    PowerDistribution pdu = new PowerDistribution(1, ModuleType.kRev);

    public void updateCurrent() {

        // Send power information
        SmartDashboard.putNumber("Voltage", pdu.getVoltage());
        SmartDashboard.putNumber("Total Current", pdu.getTotalCurrent());

        SmartDashboard.putNumber("Drive R1 Current", pdu.getCurrent(9));
        SmartDashboard.putNumber("Drive R2 Current", pdu.getCurrent(8));
        SmartDashboard.putNumber("Drive L1 Current", pdu.getCurrent(18));
        SmartDashboard.putNumber("Drive L2 Current", pdu.getCurrent(19));
    }

}
