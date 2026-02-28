package frc.robot.classes;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.abstractions.ISparkMaxMotor;

public class SparkFLEXMotorSim implements ISparkMaxMotor {

    private double maxSpeed;
    private double maxVoltage;
    private String subsystem;
    private SparkFlexSim simMotor;
    private SparkFLEXMotor motor;
    private final boolean isInverted;
    private final Designation designation;

    public SparkFLEXMotorSim(String subsystem, SparkFLEXMotor motor, double maxVoltage, double maxSpeed) {
        simMotor = new SparkFlexSim(motor.motor, DCMotor.getNEO(1));
        this.motor = motor;
        this.isInverted = motor.config.isInverted;
        this.designation = motor.config.designation;
        this.maxVoltage = maxVoltage;
        this.maxSpeed = maxSpeed;
        this.subsystem = subsystem;
    }

    public SparkFLEXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            Designation designation, double maxVoltage, double maxSpeed) {

        this(subsystem,
                new SparkFLEXMotor(subsystem, channels, isInverted, designation),
                maxVoltage,
                maxSpeed);
    }

    public static SparkMAXMotorSim CreateSparkMAXMotorSim(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation, double maxVoltage, double maxSpeed) {
        return new SparkMAXMotorSim(subsystem, channels, isInverted, designation, maxVoltage, maxSpeed);
    }

    public void simulationPeriodic(double velocity) {
        double appliedOutput = motor.getAppliedOutput(); // -1.0 to 1.0
        double deltaVelocity = appliedOutput * maxSpeed; 

        //Apply simulated values
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
        //motor.setVoltage(12);
        motor.setVelocity(velocity);
    }
}
