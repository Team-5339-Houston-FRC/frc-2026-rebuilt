package frc.robot.classes;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.abstractions.ISparkMaxMotor;

public class SparkFLEXMotor extends SparkBaseMotor<SparkFlex> implements ISparkMaxMotor {

    public SparkFLEXMotor() {

    }

    public SparkFLEXMotor(String subsystem, int channelA, boolean isInverted,
            Designation designation, double maxSpeed, double maxVoltage) {
        this(subsystem, new SparkBaseMotorChannels(channelA), isInverted, designation, maxSpeed, maxVoltage);
    }

    public SparkFLEXMotor(String subsystem, SparkBaseMotorChannels channels, boolean isInverted,
            Designation designation, double maxSpeed, double maxVoltage) {
        super(subsystem,
                channels,
                isInverted, designation, maxSpeed, maxVoltage);
    }

    public SparkFLEXMotor(SparkBaseMotorConfig<SparkFlex> config) {
        super(config);
    }

    @Override
    protected SparkFlex CreateMotor(SparkBaseMotorConfig<SparkFlex> config) {
        SparkFlex motor = new SparkFlex(config.channels.channelA, MotorType.kBrushless);
        return motor;
    }

    public void simulationPeriodic(double velocity) {

    }
}
