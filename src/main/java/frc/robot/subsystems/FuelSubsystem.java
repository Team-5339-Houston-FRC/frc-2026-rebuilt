package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Robot;
import frc.robot.abstractions.ISparkMaxMotor;
import frc.robot.classes.Designation;
import frc.robot.classes.SparkBaseMotorChannels;
import frc.robot.classes.SparkFLEXMotor;
import frc.robot.classes.SparkFLEXMotorSim;

public class FuelSubsystem extends SubsystemBase {
    private ISparkMaxMotor m_primaryMotor;
    private ISparkMaxMotor m_secondaryMotor;

    public FuelSubsystem() {
        if (Robot.isSimulation()) {
            m_primaryMotor = new SparkFLEXMotorSim("Fuel", new SparkBaseMotorChannels(FuelConstants.primaryChannelA),
                    false, Designation.Primary, FuelConstants.kMaxVoltgage, FuelConstants.kMaxSpeedMetersPerSecond);
            m_secondaryMotor = new SparkFLEXMotorSim("Fuel",
                    new SparkBaseMotorChannels(FuelConstants.secondaryChannelA),
                    false, Designation.Secondary, FuelConstants.kMaxVoltgage, FuelConstants.kMaxSpeedMetersPerSecond);
        } else {
            m_primaryMotor = new SparkFLEXMotor("Fuel", FuelConstants.primaryChannelA,
                    false,
                    Designation.Primary, FuelConstants.kMaxSpeedMetersPerSecond, FuelConstants.kMaxVoltgage);
            m_secondaryMotor = new SparkFLEXMotor("Fuel",
                    FuelConstants.secondaryChannelA,
                    false,
                    Designation.Secondary, FuelConstants.kMaxSpeedMetersPerSecond, FuelConstants.kMaxVoltgage);
        }

        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    public void intake() {
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        m_secondaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
    }

    public void shoot() {
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        if (m_primaryMotor.getVelocity() > FuelConstants.kMaxSpeedMetersPerSecond
                * FuelConstants.shootingThresholdPercent) {
            m_secondaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        }
    }

    //ball outfeed - empty hopper
    public void deballonize() {
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        m_secondaryMotor.setVelocity(-FuelConstants.kMaxSpeedMetersPerSecond);
    }

    public void stop() {
        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    @Override
    public void simulationPeriodic() {
        double velocity = m_primaryMotor.getVelocity();

        m_primaryMotor.simulationPeriodic(velocity);
        m_secondaryMotor.simulationPeriodic(velocity);
    }
}
