package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMAXMotorSim {
    private SparkMaxSim simMotor;
    private SparkMAXMotor motor;

    public SparkMAXMotorSim(int channelA, int channelB, boolean isInverted) {
        motor = new SparkMAXMotor(channelA, channelB, isInverted);
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
    }

    public SparkMAXMotorSim(SparkMAXMotor motor) {
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.motor = motor;
    }

    public SparkMAXMotorSim(SparkBaseMotorChannels channels, boolean isInverted) {
        motor = new SparkMAXMotor(channels, isInverted);
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(SparkBaseMotorChannels channels, boolean isInverted) {
        return new SparkMAXMotorSim(channels, isInverted);
    }

    public void simulationPeriodic(double velocity, double vbus, double dt) {
        simMotor.iterate(velocity, vbus, dt);
        record();
    }

    public double getVoltage() {
        return simMotor.getAppliedOutput() * simMotor.getBusVoltage();
    }

    public void record() {
        motor.record();
        String subSystem = "Drive/Sim";
        String name = String.valueOf(motor.config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        int positionCoefficient = 1; // positionCoefficient();
        SmartDashboard.putNumber(path + "Position", simMotor.getPosition());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * simMotor.getVelocity());
        SmartDashboard.putNumber(path + "Current", simMotor.getMotorCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", simMotor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
    }

    public void setDistance(double distance) {
        simMotor.setPosition(distance);
    }

    public void setVelocity(double velocity) {
        double voltage = 12;
        if (Math.abs(velocity) > 0) {
            voltage = 12;
        }
        // this.motor.setVoltage(voltage);
        // this.motor.set(velocity);
        simMotor.setVelocity(velocity);
        motor.setVelocity(velocity);
    }
}
