package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.abstractions.ISparkMaxMotor;

public class SparkMAXMotorSim implements ISparkMaxMotor {

    private double maxSpeed;
    private double maxVoltage;
    private String subsystem;
    private SparkMaxSim simMotor;
    private SparkMAXMotor motor;
    private final boolean isInverted;
    private final Designation designation;

    public SparkMAXMotorSim(String subsystem, SparkMAXMotor motor, double maxVoltage, double maxSpeed) {
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.motor = motor;
        this.isInverted = motor.config.isInverted;
        this.designation = motor.config.designation;
        this.maxVoltage = maxVoltage;
        this.maxSpeed = maxSpeed;
        this.subsystem = subsystem;
    }

    public SparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxVoltage, double maxSpeed) {
        this(subsystem, new SparkMAXMotor(subsystem, channels, isInverted, designation), maxVoltage, maxSpeed);
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation, double maxVoltage, double maxSpeed) {
        return new SparkMAXMotorSim(subsystem, channels, isInverted, designation, maxVoltage, maxSpeed);
    }

    public void simulationPeriodic(double velocity) {
        // double appliedOutput = getAppliedOutput(); // -1.0 to 1.0
        // double deltaVelocity = appliedOutput * maxSpeed * velocity;
        // double rotationsPerSecond = this.maxSpeed / 60.0;
        double voltage = maxVoltage;
        if (Math.abs(velocity) == 0) {
            velocity = 0;
        }

        double deltaVelocity = maxSpeed * velocity;
        double deltaTime = 0.02; // 20ms loop

        simMotor.iterate(deltaVelocity, voltage, deltaTime);
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
