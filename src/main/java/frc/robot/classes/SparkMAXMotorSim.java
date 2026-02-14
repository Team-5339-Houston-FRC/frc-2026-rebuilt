package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMAXMotorSim extends SparkMAXMotor {
    private SparkMaxSim simMotor;

    public SparkMAXMotorSim(int channelA, int channelB, boolean isInverted) {
        super(channelA, channelB, isInverted);
        simMotor = new SparkMaxSim(this.motor, DCMotor.getNEO(1));
    }

    public SparkMAXMotorSim(SparkBaseMotorChannels channels, boolean isInverted) {
        super(channels, isInverted);
        simMotor = new SparkMaxSim(this.motor, DCMotor.getNEO(1));
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(SparkBaseMotorChannels channels, boolean isInverted) {
        return new SparkMAXMotorSim(channels, isInverted);
    }
    
    public void simulationPeriodic(double velocity, double vbus, double dt) {
        setVelocity(velocity);
        simMotor.iterate(velocity, vbus, dt);
        record();
    }

    @Override
    public void record() {
        super.record();
        String subSystem = "Drive/Sim";
        String name = String.valueOf(config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        int positionCoefficient = 1; // positionCoefficient();
        SmartDashboard.putNumber(path + "Position", simMotor.getPosition());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * simMotor.getVelocity());
        SmartDashboard.putNumber(path + "Current", simMotor.getMotorCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", simMotor.getBusVoltage());
    }

    public void setDistance(double distance) {
        simMotor.setPosition(distance);
    }

    public void setVelocity(double velocity) {
        double voltage = 12;
        if (Math.abs(velocity) > 0) {
            voltage = 12;
        }
        this.motor.setVoltage(voltage);
        simMotor.setVelocity(velocity);
    }
}
