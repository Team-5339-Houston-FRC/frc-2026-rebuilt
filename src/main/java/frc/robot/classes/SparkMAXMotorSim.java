package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.abstractions.ISparkMaxMotor;

public class SparkMAXMotorSim implements ISparkMaxMotor {

    private String subsystem;
    private SparkMaxSim simMotor;
    private SparkMAXMotor motor;
    private final boolean isInverted;
    private final Designation designation;
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public SparkMAXMotorSim(SparkMAXMotor motor) {
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.motor = motor;
        this.isInverted = motor.config.isInverted;
        this.designation = motor.config.designation;

    }

    public SparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels, boolean isInverted,
            Designation designation) {
        this.subsystem = subsystem;
        motor = new SparkMAXMotor(subsystem, channels, isInverted, designation);
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.isInverted = isInverted;
        this.designation = designation;
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation) {
        return new SparkMAXMotorSim(subsystem, channels, isInverted, designation);
    }

    public void simulationPeriodic(double velocity, double vbus, double dt) {   
        double appliedOutput = motor.getAppliedOutput(); // -1.0 to 1.0
        double deltaPosition = appliedOutput * 0.05; // arbitrary units per loop
        double deltaVelocity = appliedOutput * 1500;  // arbitrary RPM

        // // Apply simulated values
        // simMotor.getRelativeEncoderSim().setPosition(motor.getDistance() + 5);
        // simMotor.getRelativeEncoderSim().setVelocity(deltaVelocity);

        // Optionally simulate current draw
        simMotor.setBusVoltage(12.0);
        simMotor.iterate(deltaVelocity, vbus, dt);
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
        int positionCoefficient = 1; // positionCoefficient();
        SmartDashboard.putNumber(path + "Position", simMotor.getRelativeEncoderSim().getPosition());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * simMotor.getRelativeEncoderSim().getVelocity());
        SmartDashboard.putNumber(path + "Current", simMotor.getMotorCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", simMotor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
        SmartDashboard.putBoolean(path + "IsInverted", isInverted);
        SmartDashboard.putString(path + "Designation", designation.toString());
    }

    // public void setDistance(double distance) {
    // simMotor.setPosition(distance);
    // }

    public void setVelocity(double velocity) {
        motor.setVoltage(12);
        motor.setVelocity(velocity);
        simMotor.setVelocity(velocity);
    }
}
