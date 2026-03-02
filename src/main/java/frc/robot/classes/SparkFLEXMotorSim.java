package frc.robot.classes;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
public class SparkFLEXMotorSim extends SparkMotorSim<SparkFlex, SparkFlexSim> {

    public SparkFLEXMotorSim(String subsystem, SparkFLEXMotor motor, double maxVoltage, double maxSpeed) {
        super(motor, new SparkFlexSim(motor.motor, DCMotor.getNEO(1)), motor.config);
    }

    public SparkFLEXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxVoltage, double maxSpeed) {
        this(subsystem, new SparkFLEXMotor(subsystem, channels, isInverted, designation, maxSpeed, maxVoltage),
                maxVoltage, maxSpeed);
    }   

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation, double maxVoltage, double maxSpeed) {
        return new SparkMAXMotorSim(subsystem, channels, isInverted, designation, maxVoltage, maxSpeed);
    }

    @Override
    protected SparkBaseMotor<SparkFlex> CreateMotor(SparkBaseMotorConfig<SparkFlex> config) {
        return new SparkFLEXMotor(config.subSystem, config.channels, config.isInverted, config.designation,
                config.maxVoltage, config.maxSpeed);
    }

    @Override
    protected SparkFlexSim CreateMotorSim(SparkBaseMotor<SparkFlex> motor, SparkBaseMotorConfig<SparkFlex> config) {
        return new SparkFlexSim(motor.motor, DCMotor.getNEO(1));
    }
}
