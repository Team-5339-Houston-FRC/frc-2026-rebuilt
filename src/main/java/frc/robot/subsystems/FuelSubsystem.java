package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.Robot;
import frc.robot.abstractions.ISparkMaxMotor;
import frc.robot.classes.Designation;
import frc.robot.classes.SparkBaseMotorChannels;
import frc.robot.classes.SparkMAXMotor;
import frc.robot.classes.SparkMAXMotorSim;
import frc.robot.classes.SparkMaxMotorArray;
import frc.robot.classes.SparkMaxMotorArraySim;

public class FuelSubsystem extends SubsystemBase {
    private ISparkMaxMotor m_primaryMotor;
    private ISparkMaxMotor m_secondaryMotor;

    public FuelSubsystem() {
        if (Robot.isSimulation()) {
            m_primaryMotor = new SparkMAXMotorSim("Fuel", new SparkBaseMotorChannels(FuelConstants.primaryChannelA),
                    false, Designation.Primary);
            m_secondaryMotor = new SparkMAXMotorSim("Fuel", new SparkBaseMotorChannels(FuelConstants.secondaryChannelA),
                    false, Designation.Secondary);
        } else {
            m_primaryMotor = new SparkMAXMotor("Fuel", FuelConstants.primaryChannelA,
                    false,
                    Designation.Primary);
            m_secondaryMotor = new SparkMAXMotor("Fuel",
                    FuelConstants.secondaryChannelB,
                    false,
                    Designation.Secondary);
        }

        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    public void intake() {
        m_primaryMotor.setVelocity(FuelConstants.maxSpeed);
        m_secondaryMotor.setVelocity(FuelConstants.maxSpeed);
    }

    public void shoot() {
        m_primaryMotor.setVelocity(FuelConstants.maxSpeed);
        if (m_primaryMotor.getVelocity() > FuelConstants.maxSpeed * FuelConstants.shootingThresholdPercent) {
            m_secondaryMotor.setVelocity(-1 * FuelConstants.maxSpeed);
        }
    }

    // decolonize but balls
    public void deballonize() {
        m_primaryMotor.setVelocity(-1 * FuelConstants.maxSpeed);
        m_secondaryMotor.setVelocity(-1 * FuelConstants.maxSpeed);
    }

    public void stop() {
        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }

    @Override
    public void simulationPeriodic() {
        double rotationsPerSecond = 1500 / 60.0;
        double deltaTime = 0.02; // 20ms loop
        double deltaRotations = rotationsPerSecond * deltaTime;
        double velocity = m_primaryMotor.getVelocity();

        m_primaryMotor.simulationPeriodic(velocity, 12, deltaRotations);
        m_secondaryMotor.simulationPeriodic(velocity, 12, deltaRotations);
    }
}
