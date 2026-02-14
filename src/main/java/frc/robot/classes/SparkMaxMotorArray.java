package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.abstractions.ISparkMaxMotorArray;
import frc.robot.classes.SparkMaxMotorArraySim.Designation;

public class SparkMaxMotorArray implements ISparkMaxMotorArray {

    protected final List<SparkMAXMotor> motors = new ArrayList<SparkMAXMotor>();
    public double distance;

    public SparkMaxMotorArray() {

    }

    public SparkMaxMotorArray(List<SparkBaseMotorChannels> channels, boolean isInverted) {
        SparkMAXMotor leader = null;
        for (SparkBaseMotorChannels channel : channels) {
            SparkBaseMotorConfig<SparkMax> config = new SparkBaseMotorConfig<SparkMax>(channel, isInverted,
                    0, leader);
            SparkMAXMotor motor = new SparkMAXMotor(config);
            if (leader == null) {
                leader = motor;
            }
            motors.add(motor);
        }
    }

    public double getDistance() {
        SparkMAXMotor leader = motors.get(0);
        return leader.getDistance();
    }

    public SparkMAXMotor getLeader() {
        SparkMAXMotor leader = motors.get(0);
        return leader;
    }

    public double getVoltage() {
        SparkMAXMotor leader = motors.get(0);
        return leader.getVoltage();
    }

    public void setDistance(double position) {
        distance = position;
    }

    public void simulationPeriodic(double velocity, double distance) {

    }

    public void setSpeeds(double metersPerSecond, double feedforward) {
        SparkMAXMotor leader = motors.get(0);
        leader.setSpeeds(metersPerSecond, feedforward);
    }
}
