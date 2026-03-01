package frc.robot.classes;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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
        pidController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        if (config.isInverted) {
            velocityCoefficient = -1;
        }

        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.inverted(config.isInverted);

        if (config.leader != null) {
            sparkMaxConfig.follow(config.leader.motor);
        }

        motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public SparkBaseMotor(String subsystem, SparkBaseMotorChannels channels, boolean isInverted,
            Designation designation, double maxSpeed, double maxVoltage) {
        this(new SparkBaseMotorConfig<T>(
                subsystem,
                new SparkBaseMotorChannels(channels.channelA, channels.channelB),
                isInverted, designation, maxSpeed, maxVoltage));
    }

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

    public void setVelocity(double speed) {
        double motorSpeed = velocityCoefficient * (speed / FuelConstants.kMaxSpeedMetersPerSecond);// RPM Value Scaled
                                                                                                   // to -1->1
        if (Math.abs(motorSpeed) == 0) {
            motorSpeed = 0;
            motor.setVoltage(0);
            motor.set(motorSpeed);
            motor.stopMotor();
        } else {
            motor.setVoltage(12);
            motor.set(motorSpeed);
        }
        // pidController.setSetpoint(motorSpeed, ControlType.kVelocity);
    }

    public double getDistance() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public void record() {
        String subSystem = config.subSystem;
        String name = String.valueOf(config.channels.channelA);
        String path = subSystem + "/" + name + "/";

        SmartDashboard.putNumber(path + "Position", encoder.getPosition());
        SmartDashboard.putNumber(path + "Velocity", encoder.getVelocity());
        SmartDashboard.putNumber(path + "Current", motor.getOutputCurrent());
        SmartDashboard.putNumber(path + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(path + "Voltage", getVoltage());
        SmartDashboard.putBoolean(path + "IsInverted", config.isInverted);
        SmartDashboard.putString(path + "Designation", config.designation.toString());
    }

}
