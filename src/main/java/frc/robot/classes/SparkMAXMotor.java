package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SparkMAXMotor extends MotorBase {

    public SparkMAXMotor(int channelA, int channelB, boolean isInverted) {
        super(channelA, channelB, isInverted);
    }

    public void addFollower(SparkMAXMotor follower) {
        this.addFollower(follower);
    }

    public static SparkMAXMotor CreateSparkMaxMotor(int channelA, int channelB, boolean isInverted) {
        return new SparkMAXMotor(channelA, channelB, isInverted);
    }

    @SuppressWarnings("deprecation")
    @Override
    protected SparkMax CreateMotor(int channel, boolean isInverted) {
        SparkMax l = new SparkMax(channel, MotorType.kBrushless);
        l.setInverted(isInverted);
        
        return l;
    }

    public SparkMax getMotor() {
        return (SparkMax) super._leader;
    }

    public double get() {
        return getMotor().get();
    }
}
