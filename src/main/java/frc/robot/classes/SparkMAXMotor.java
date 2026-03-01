package frc.robot.classes;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.abstractions.ISparkMaxMotor;

public class SparkMAXMotor extends SparkBaseMotor<SparkMax> implements ISparkMaxMotor {

    public SparkMAXMotor() {

    }

    public SparkMAXMotor(String subsystem, int channelA, boolean isInverted,
     Designation designation, double maxSpeed, double maxVoltage) {
        this(subsystem,
                new SparkBaseMotorChannels(channelA),
                isInverted, designation, maxSpeed, maxVoltage);
    }

    public SparkMAXMotor(String subsystem,
            SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxSpeed, double maxVoltage) {
        super(subsystem,
                channels,
                isInverted, designation, maxSpeed, maxVoltage);
    }

    public SparkMAXMotor(SparkBaseMotorConfig<SparkMax> config) {
        super(config);
    }

    @Override
    protected SparkMax CreateMotor(SparkBaseMotorConfig<SparkMax> config) {
        SparkMax motor = new SparkMax(config.channels.channelA, MotorType.kBrushless);
        return motor;
    }

    public void simulationPeriodic(double velocity) {

    }
}
