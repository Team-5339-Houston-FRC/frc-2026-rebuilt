package frc.robot.classes;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FuelConstants;

public abstract class SparkBaseMotor<T extends SparkBase> {

    public T motor;
    protected RelativeEncoder encoder;
    protected SparkClosedLoopController pidController;
    protected SparkBaseMotorConfig<T> config;
    private int velocityCoefficient = 1;

    public SparkBaseMotor() {

    }

    public SparkBaseMotor(SparkBaseMotorConfig<T> config) {
        this.config = config;
        motor = CreateMotor(config);
        //motor.setVoltage(12);
        pidController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        if (config.isInverted) {
            velocityCoefficient = -1;
        }
    }

    public SparkBaseMotor(String subsystem, SparkBaseMotorChannels channels, boolean isInverted,
            Designation designation) {
        this(new SparkBaseMotorConfig<T>(
                subsystem,
                new SparkBaseMotorChannels(channels.channelA, channels.channelB),
                isInverted, designation));
    }

    protected abstract T CreateMotor(int channel, boolean isInverted);

    protected abstract T CreateMotor(SparkBaseMotorConfig<T> config);

    public double get() {
        return motor.get();
    }

    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    // set the motor's speed
    public void setVelocity(double speed) {
        pidController.setSetpoint(speed, ControlType.kVelocity);
        double motorSpeed = speed / FuelConstants.maxSpeed;//RPM Value Scaled to -1->1
        motor.set(motorSpeed * velocityCoefficient);
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    public void record() {
        String subSystem = config.subSystem;
        String name = String.valueOf(config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        int positionCoefficient = 1; // positionCoefficient();

        SmartDashboard.putNumber(path + "Position", encoder.getPosition());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * encoder.getVelocity());
        SmartDashboard.putNumber(path + "Current", motor.getOutputCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
        SmartDashboard.putBoolean(path + "IsInverted", config.isInverted);
        SmartDashboard.putString(path + "Designation", config.designation.toString());
    }

}
