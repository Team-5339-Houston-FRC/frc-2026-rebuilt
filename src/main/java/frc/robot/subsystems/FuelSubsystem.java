package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;
import frc.robot.classes.SparkMAXMotor;

public class FuelSubsystem extends SubsystemBase {
    private final SparkMAXMotor m_primaryMotor;
    private final SparkMAXMotor m_secondaryMotor;

    public FuelSubsystem() {
        m_primaryMotor = new SparkMAXMotor(FuelConstants.primaryChannelA, FuelConstants.primaryChannelB, false);
        m_secondaryMotor = new SparkMAXMotor(FuelConstants.secondaryChannelA, FuelConstants.secondaryChannelB, false);
    }

    public void intake() {
        m_primaryMotor.setVelocity(FuelConstants.maxSpeed);
        m_secondaryMotor.setVelocity(FuelConstants.maxSpeed);
    }

    public void shoot() {
        m_primaryMotor.setVelocity(FuelConstants.maxSpeed);
        m_secondaryMotor.setVelocity(-1*FuelConstants.maxSpeed);
    }

    //decolonize but balls
    public void deballonize() {
        m_primaryMotor.setVelocity(-1*FuelConstants.maxSpeed);
        m_secondaryMotor.setVelocity(-1*FuelConstants.maxSpeed);
    }

    public void stop() {
        m_primaryMotor.setVelocity(0);
        m_secondaryMotor.setVelocity(0);
    }
}
