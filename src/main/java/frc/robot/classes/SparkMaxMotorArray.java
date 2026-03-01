package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.abstractions.ISparkMaxMotorArray;

public class SparkMaxMotorArray implements ISparkMaxMotorArray {

    protected final List<SparkMAXMotor> motors = new ArrayList<SparkMAXMotor>();
    public double distance;
    private SparkMAXMotor leader;

    public SparkMaxMotorArray() {

    }

    public SparkMaxMotorArray(String subsystem,
            List<SparkBaseMotorChannels> channels, 
            boolean isInverted,
            Designation designation, double maxSpeed, double maxVoltage) {

        for (SparkBaseMotorChannels channel : channels) {
            SparkBaseMotorConfig<SparkMax> config = new SparkBaseMotorConfig<SparkMax>(subsystem, channel, isInverted,
                    designation, maxSpeed, maxVoltage);
            SparkMAXMotor motor = new SparkMAXMotor(config);
            if (leader == null) {
                leader = motor;
            }
            motors.add(motor);
        }
    }

    public double getDistance() {
        return leader.getDistance();
    }

    public SparkMAXMotor getLeader() {
        return leader;
    }

    public double getVoltage() {
        return leader.getVoltage();
    }

    public void setDistance(double position) {
        distance = position;
    }

    public void setVelocity(double velocity) {
        leader.setVelocity(velocity);
    }

    public void simulationPeriodic(double velocity) {

    }
}
