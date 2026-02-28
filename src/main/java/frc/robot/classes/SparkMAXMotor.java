package frc.robot.classes;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.abstractions.ISparkMaxMotor;

public class SparkMAXMotor extends SparkBaseMotor<SparkMax> implements ISparkMaxMotor {

    public SparkMAXMotor() {

    }

    public SparkMAXMotor(String subsystem, int channelA, boolean isInverted, Designation designation) {
        this(subsystem,
                new SparkBaseMotorChannels(channelA),
                isInverted, designation);
    }

    public SparkMAXMotor(String subsystem,
            SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation) {
        super(subsystem,
                channels,
                isInverted, designation);
    }

    public SparkMAXMotor(SparkBaseMotorConfig<SparkMax> config) {
        super(config);
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

    public void simulationPeriodic(double velocity) {

    }
}
