package frc.robot.classes;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.abstractions.ISparkMaxMotor;

public abstract class SparkMotorSim<TMotor extends SparkBase, TMotorSim extends SparkSim> implements ISparkMaxMotor {

    private double maxSpeed;
    private double maxVoltage;
    private String subsystem;
    private TMotorSim simMotor;
    private SparkBaseMotor<TMotor> motor;
    private boolean isInverted;
    private Designation designation;

    public SparkMotorSim(SparkBaseMotor<TMotor> motor, TMotorSim motorSim, SparkBaseMotorConfig<TMotor> config) {
        this.simMotor = motorSim;
        this.motor = motor;
        configure(config);
    }

    private void configure(SparkBaseMotorConfig<TMotor> config) {
        this.isInverted = config.isInverted;
        this.designation = config.designation;
        this.maxVoltage = config.maxVoltage;
        this.maxSpeed = config.maxSpeed;
        this.subsystem = config.subSystem;
    }

    protected abstract SparkBaseMotor<TMotor> CreateMotor(SparkBaseMotorConfig<TMotor> config);

    protected abstract TMotorSim CreateMotorSim(SparkBaseMotorConfig<TMotor> config);

    public SparkMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxVoltage, double maxSpeed) {
        SparkBaseMotorConfig<TMotor> config = new SparkBaseMotorConfig<TMotor>(
                subsystem,
                new SparkBaseMotorChannels(channels.channelA, channels.channelB),
                isInverted, designation,
                maxVoltage,
                maxSpeed);

        this.simMotor = CreateMotorSim(config);
        this.motor = CreateMotor(config);
        configure(config);

    }

    public void simulationPeriodic(double velocity) {
        double appliedOutput = motor.getAppliedOutput(); // -1.0 to 1.0
        double deltaVelocity = appliedOutput * maxSpeed;

        // Apply simulated values
        // simMotor.getRelativeEncoderSim().setPosition(motor.getDistance() + 5);
        // simMotor.getRelativeEncoderSim().setVelocity(deltaVelocity);

        double deltaTime = 0.02; // 20ms loop
        simMotor.iterate(deltaVelocity, maxVoltage, deltaTime);
        record();
    }

    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void record() {
        motor.record();
        String subSystem = this.subsystem + "/Sim";
        String name = String.valueOf(motor.config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        SmartDashboard.putNumber(path + "Position", simMotor.getRelativeEncoderSim().getPosition());
        SmartDashboard.putNumber(path + "Velocity", simMotor.getRelativeEncoderSim().getVelocity());
        SmartDashboard.putNumber(path + "Current", simMotor.getMotorCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", simMotor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
        SmartDashboard.putBoolean(path + "IsInverted", isInverted);
        SmartDashboard.putString(path + "Designation", designation.toString());
    }

    public void setVelocity(double velocity) {
        motor.setVelocity(velocity);
    }
}
