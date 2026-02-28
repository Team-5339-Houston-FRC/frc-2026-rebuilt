package frc.robot.classes;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.abstractions.ISparkMaxMotor;

public class SparkFLEXMotor extends SparkBaseMotor<SparkFlex> implements ISparkMaxMotor {

    public SparkFLEXMotor() {

    }

    public SparkFLEXMotor(String subsystem, int channelA, boolean isInverted, Designation designation) {
        this(subsystem, new SparkBaseMotorChannels(channelA), isInverted, designation);
    }

    public SparkFLEXMotor(String subsystem, SparkBaseMotorChannels channels, boolean isInverted,
            Designation designation) {
        super(subsystem,
                channels,
                isInverted, designation);
    }

    public SparkFLEXMotor(SparkBaseMotorConfig<SparkFlex> config) {
        super(config);
    }

    @Override
    protected SparkFlex CreateMotor(SparkBaseMotorConfig<SparkFlex> config) {
        SparkFlex motor = new SparkFlex(config.channels.channelA, MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(config.isInverted);

        if (config.leader != null) {
            sparkMaxConfig.follow(config.leader.motor);
        }

        motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        return motor;
    }

    public void simulationPeriodic(double velocity) {

    }
}
