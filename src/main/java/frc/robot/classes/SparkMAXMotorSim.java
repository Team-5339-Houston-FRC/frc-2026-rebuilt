package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMAXMotorSim {

    private SparkMaxSim simMotor;
    private SparkMAXMotor motor;
    private final boolean isInverted;
    private final Designation designation;
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public SparkMAXMotorSim(int channelA, int channelB, boolean isInverted, Designation designation) {
        motor = new SparkMAXMotor(channelA, channelB, isInverted, designation);
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.isInverted = isInverted;
        this.designation =  designation;
    }

    public SparkMAXMotorSim(SparkMAXMotor motor) {
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.motor = motor;
        this.isInverted = motor.config.isInverted;
        this.designation =  motor.config.designation;
    }

    public SparkMAXMotorSim(SparkBaseMotorChannels channels, boolean isInverted, Designation designation) {
        motor = new SparkMAXMotor(channels, isInverted, designation);
        simMotor = new SparkMaxSim(motor.motor, DCMotor.getNEO(1));
        this.isInverted = isInverted;
        this.designation =  designation;
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(SparkBaseMotorChannels channels, 
    boolean isInverted, Designation designation) {
        return new SparkMAXMotorSim(channels, isInverted, designation);
    }

    public void simulationPeriodic(double velocity, double vbus, double dt) {
        simMotor.iterate(velocity, vbus, dt);
        record();
    }

    public double getVoltage() {
        return motor.getVoltage();
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
        SmartDashboard.putBoolean(path + "IsInverted", isInverted);
        SmartDashboard.putString(path + "Designation", designation.toString());
    }

    // public void setDistance(double distance) {
    //     simMotor.setPosition(distance);
    // }

    // public void setVelocity(double velocity) {
    //     motor.setVoltage(12);
    //     simMotor.setVelocity(velocity);
    //     motor.setVelocity(velocity);
    // }
}
