package frc.robot.subsystems;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Robot;
import frc.robot.abstractions.ISparkMaxMotor;
import frc.robot.classes.Designation;
import frc.robot.classes.SparkBaseMotorChannels;
import frc.robot.classes.SparkFLEXMotor;
import frc.robot.classes.SparkFLEXMotorSim;

public class FuelSubsystem extends SubsystemBase {

    private final StringPublisher marqueePublisher;
    private final String subsystem = "Fuel";
    private ISparkMaxMotor m_primaryMotor;
    private ISparkMaxMotor m_secondaryMotor;

    public FuelSubsystem(StringPublisher marqueePublisher) {
        this.marqueePublisher = marqueePublisher;

        if (Robot.isSimulation()) {
            m_primaryMotor = new SparkFLEXMotorSim(subsystem, new SparkBaseMotorChannels(FuelConstants.primaryChannelA),
                    false, Designation.Primary, FuelConstants.kMaxVoltgage, FuelConstants.kMaxSpeedMetersPerSecond);
            m_secondaryMotor = new SparkFLEXMotorSim(subsystem,
                    new SparkBaseMotorChannels(FuelConstants.secondaryChannelA),
                    false, Designation.Secondary, FuelConstants.kMaxVoltgage, FuelConstants.kMaxSpeedMetersPerSecond);
        } else {
            m_primaryMotor = new SparkFLEXMotor(subsystem, FuelConstants.primaryChannelA,
                    false,
                    Designation.Primary, FuelConstants.kMaxSpeedMetersPerSecond, FuelConstants.kMaxVoltgage);
            m_secondaryMotor = new SparkFLEXMotor(subsystem,
                    FuelConstants.secondaryChannelA,
                    false,
                    Designation.Secondary, FuelConstants.kMaxSpeedMetersPerSecond, FuelConstants.kMaxVoltgage);
        }

        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    public void intake() {
        marqueePublisher.set("Intake");
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        m_secondaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
    }

    public void shoot() {
        marqueePublisher.set("Shoot");
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        if (m_primaryMotor.getVelocity() > FuelConstants.kMaxSpeedMetersPerSecond
                * FuelConstants.shootingThresholdPercent) {
            m_secondaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        }
    }

    // ball outfeed - empty hopper
    public void deballonize() {
        marqueePublisher.set("Outfeed");
        m_primaryMotor.setVelocity(FuelConstants.kMaxSpeedMetersPerSecond);
        m_secondaryMotor.setVelocity(-FuelConstants.kMaxSpeedMetersPerSecond);
    }

    public void stop() {
        
        marqueePublisher.set("Ready");
        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    @Override
    public void simulationPeriodic() {
        m_primaryMotor.simulationPeriodic(m_primaryMotor.getVelocity());
        m_secondaryMotor.simulationPeriodic(m_secondaryMotor.getVelocity());
    }
}
