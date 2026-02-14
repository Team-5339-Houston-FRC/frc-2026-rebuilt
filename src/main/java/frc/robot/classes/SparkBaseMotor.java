package frc.robot.classes;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SparkBaseMotor<T extends SparkBase> {

    public T motor;
    protected Encoder encoder;
    protected SparkClosedLoopController pidController;
    protected SparkBaseMotorConfig<T> config;

    public SparkBaseMotor() {

    }

    public SparkBaseMotor(SparkBaseMotorConfig<T> config) {
        this.config = config;
        motor = CreateMotor(config);
        pidController = motor.getClosedLoopController();
        encoder = new Encoder(config.channels.channelA, config.channels.channelB);
        encoder.setDistancePerPulse(config.distancePerPulse);
    }

    public SparkBaseMotor(int channelA, int channelB, boolean isInverted, double distancePerPulse) {
        this(new SparkBaseMotorConfig<T>(
                new SparkBaseMotorChannels(channelA, channelB),
                isInverted, distancePerPulse));
    }

    public SparkBaseMotor(int channelA, int channelB, boolean isInverted) {
        this(new SparkBaseMotorConfig<T>(
                new SparkBaseMotorChannels(channelA, channelB),
                isInverted));
    }

    public SparkBaseMotor(SparkBaseMotorChannels channels, boolean isInverted) {
        this(new SparkBaseMotorConfig<T>(
                new SparkBaseMotorChannels(channels.channelA, channels.channelB),
                isInverted));
    }

    protected abstract T CreateMotor(int channel, boolean isInverted);

    protected abstract T CreateMotor(SparkBaseMotorConfig<T> config);

    public double get() {
        return motor.get();
    }

    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public void setSpeeds(double metersPerSecond, double feedforward) {
        // double output = pidController.calculate(encoder.getRate(), metersPerSecond);
        // motor.setVoltage(output + feedforward);
        pidController.setSetpoint(metersPerSecond, ControlType.kVelocity);
        pidController.setSetpoint(1, ControlType.kPosition);
        motor.setVoltage(12);
        motor.set(metersPerSecond);
    }

    // set the motor's speed
    public void setVelocity(double speed) {
        pidController.setSetpoint(speed, ControlType.kVelocity);
        pidController.setSetpoint(0, ControlType.kPosition);
        motor.setVoltage(12);
        motor.set(speed);
    }

    public double getDistance() {
        return encoder.getDistance();
    }

    public void record() {
        String subSystem = "Drive";
        String name = String.valueOf(config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        int positionCoefficient = 1; // positionCoefficient();
        SmartDashboard.putNumber(path + "Position", encoder.getDistance());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * encoder.get());
        SmartDashboard.putNumber(path + "Current", motor.getOutputCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
    }

}
