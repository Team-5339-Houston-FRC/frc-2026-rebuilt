package frc.robot.classes;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        motor.setVoltage(12);
        pidController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        if (config.isInverted) {
            velocityCoefficient = -1;
        }
    }

    public SparkBaseMotor(int channelA, int channelB, boolean isInverted,
            double distancePerPulse, Designation designation) {
        this(new SparkBaseMotorConfig<T>(
                new SparkBaseMotorChannels(channelA, channelB),
                isInverted, distancePerPulse, designation));
    }

    public SparkBaseMotor(int channelA, int channelB,
            boolean isInverted, Designation designation) {
        this(new SparkBaseMotorConfig<T>(
                new SparkBaseMotorChannels(channelA, channelB),
                isInverted, designation));
    }

    public SparkBaseMotor(SparkBaseMotorChannels channels, boolean isInverted, Designation designation) {
        this(new SparkBaseMotorConfig<T>(
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

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setSpeeds(double metersPerSecond, double feedforward) {
        // double output = pidController.calculate(encoder.getRate(), metersPerSecond);
        // motor.setVoltage(output + feedforward);
        pidController.setSetpoint(metersPerSecond, ControlType.kVelocity);
        //pidController.setSetpoint(1, ControlType.kPosition);
        motor.setVoltage(12);
        motor.set(metersPerSecond * velocityCoefficient);
    }

    // set the motor's speed
    public void setVelocity(double speed) {
        pidController.setSetpoint(speed * 1500, ControlType.kVelocity);
        //pidController.setSetpoint(0, ControlType.kPosition);
        motor.setVoltage(12);
        motor.set(speed * velocityCoefficient);
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    public void record() {
        String subSystem = "Drive";
        String name = String.valueOf(config.channels.channelA);
        String path = subSystem + "/" + name + "/";
        int positionCoefficient = 1; // positionCoefficient();

        SmartDashboard.putNumber(path + "Position", encoder.getPosition());
        SmartDashboard.putNumber(path + "Velocity", positionCoefficient * motor.get());
        SmartDashboard.putNumber(path + "Current", motor.getOutputCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
        SmartDashboard.putBoolean(path + "IsInverted", config.isInverted);
        SmartDashboard.putString(path + "Designation", config.designation.toString());
    }

}
