package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class SparkMaxMotorArraySim extends SparkMaxMotorArray {

    private final Designation designation;
    private SparkMAXMotorSim leaderSim;
    private final List<SparkMAXMotorSim> motorSims = new ArrayList<SparkMAXMotorSim>();

    public SparkMaxMotorArraySim(List<SparkBaseMotorChannels> channels, Designation designation, boolean isInverted) {
        super(channels, isInverted);
        this.designation = designation;

        for (SparkMAXMotor motor : this.motors) {
            if (leaderSim == null) {
                leaderSim = new SparkMAXMotorSim(motor);
                motorSims.add(leaderSim);
            } else {
                SparkMAXMotorSim follower = new SparkMAXMotorSim(motor);
                motorSims.add(follower);
            }
        }
    }

    // public void setSpeeds(double metersPerSecond, double feedforward) {
    // super.setSpeeds(metersPerSecond, feedforward);
    // }

    @Override
    public void simulationPeriodic(double velocity, double distance) {
        double rotationsPerSecond = 1500 / 60.0;
        double deltaTime = 0.02; // 20ms loop
        double deltaRotations = rotationsPerSecond * deltaTime;

        setDistance(distance);
        leaderSim.simulationPeriodic(velocity, deltaTime, deltaRotations);    
    }

    @Override
    public void setSpeeds(double metersPerSecond, double feedforward) {
        super.setSpeeds(metersPerSecond, feedforward);
        SparkMAXMotor leader = motors.get(0);
        leader.setSpeeds(metersPerSecond, feedforward);
    }

    @Override
    public void setDistance(double position) {
        leaderSim.setDistance(position);
    }

    public enum Designation {
        Left,
        Right
    }
}
