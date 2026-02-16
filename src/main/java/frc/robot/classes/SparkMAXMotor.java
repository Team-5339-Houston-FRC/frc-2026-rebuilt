package frc.robot.classes;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMAXMotor extends SparkBaseMotor<SparkMax> {

    public SparkMAXMotor() {

    }
    
    public SparkMAXMotor(int channelA, int channelB, boolean isInverted, Designation designation) {
        super(channelA, channelB, isInverted, designation);
    }

    public SparkMAXMotor(SparkBaseMotorChannels channels, boolean isInverted, Designation designation) {
        super(channels, isInverted, designation);
    }

    public SparkMAXMotor(SparkBaseMotorConfig<SparkMax> config) {
        super(config);
    }

    @Override
    protected SparkMax CreateMotor(int channel, boolean isInverted) {
        SparkMax motor = new SparkMax(channel, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(isInverted);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        return motor;
    }

    @Override
    protected SparkMax CreateMotor(SparkBaseMotorConfig<SparkMax> config) {
        SparkMax motor = new SparkMax(config.channels.channelA, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(config.isInverted);
        if (config.leader != null) {
            sparkMaxConfig.follow(config.leader.motor);
        }
        motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        return motor;
    }

    @Override
    public void setSpeeds(double metersPerSecond, double feedforward) {
      super.setSpeeds(metersPerSecond, feedforward);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}
