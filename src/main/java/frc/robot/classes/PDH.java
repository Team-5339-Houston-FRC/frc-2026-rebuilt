package frc.robot.classes;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PDH extends SubsystemBase {

    PowerDistribution m_pdh = new PowerDistribution(Constants.PDHConstants.PDH_CAN_ID,ModuleType.kRev);

    public void putPDHInfo() {
        /**
         * Get the input voltage of the PDH and display it on Shuffleboard.
         */
        SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());

        /**
         * Get the total current of the PDH and display it on Shuffleboard. This will
         * be to the nearest even number.
         *
         * To get a better total current reading, sum the currents of all channels.
         * See below for getting channel current.
         */
        SmartDashboard.putNumber("Total Current", m_pdh.getTotalCurrent());

        /**
         * Get the currents of each channel of the PDH and display them on
         * Shuffleboard.
         */
        SmartDashboard.putData(m_pdh);
    }

    @Override
    public void periodic() {
        putPDHInfo();
    }

    public void defaultCommand() {
        
    }
}
