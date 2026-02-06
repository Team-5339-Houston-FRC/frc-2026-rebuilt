package frc.robot.classes;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SparkMAXMotor extends MotorBase {


    public SparkMAXMotor(int channelA, int channelB, boolean isInverted) {
        super(channelA, channelB, isInverted);


    }

    @Override
    protected PWMMotorController CreateMotor(int channel, boolean isInverted) {
        PWMSparkMax l = new PWMSparkMax(channel);
        l.setInverted(isInverted);
        return l;
    }
}
