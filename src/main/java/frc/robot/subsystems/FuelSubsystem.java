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

    }

    //decolonize but balls
    public void deballonize() {

    }
}
