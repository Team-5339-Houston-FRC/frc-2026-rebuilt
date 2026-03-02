package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkMAXMotorSim extends SparkMotorSim<SparkMax, SparkMaxSim> {

    public SparkMAXMotorSim(String subsystem, SparkMAXMotor motor, double maxVoltage, double maxSpeed) {
        super(motor, new SparkMaxSim(motor.motor, DCMotor.getNEO(1)), motor.config);
    }

    public SparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxVoltage, double maxSpeed) {
        this(subsystem, new SparkMAXMotor(subsystem, channels, isInverted, designation, maxSpeed, maxVoltage),
                maxVoltage, maxSpeed);
    }

    @Override
    protected SparkBaseMotor<SparkMax> CreateMotor(SparkBaseMotorConfig<SparkMax> config) {
        return new SparkMAXMotor(config.subSystem, config.channels, config.isInverted, config.designation,
                config.maxVoltage, config.maxSpeed);
    }

    @Override
    protected SparkMaxSim CreateMotorSim(SparkBaseMotor<SparkMax> motor, SparkBaseMotorConfig<SparkMax> config) {
        return new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
    }
}
