package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

public class SparkMaxMotorArraySim extends SparkMaxMotorArray {

    private SparkMAXMotorSim leaderSim;
    private final List<SparkMAXMotorSim> motorSims = new ArrayList<SparkMAXMotorSim>();

    public SparkMaxMotorArraySim(List<SparkBaseMotorChannels> channels, Designation designation, boolean isInverted) {
        super(channels, isInverted, designation);

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

    @Override
    public void simulationPeriodic(double velocity, double distance) {
        double rotationsPerSecond = 1500 / 60.0;
        double deltaTime = 0.02; // 20ms loop
        double deltaRotations = rotationsPerSecond * deltaTime;
        SparkMAXMotor leader = motors.get(0);
        double voltage = 12 * velocity;
        setDistance(distance);
        leaderSim.simulationPeriodic(velocity, 12, deltaRotations);
    }

    @Override
    public void setSpeeds(double metersPerSecond, double feedforward) {
        super.setSpeeds(metersPerSecond, feedforward);
        SparkMAXMotor leader = motors.get(0);
        leader.setSpeeds(metersPerSecond, feedforward);
    }

    // @Override
    // public void setDistance(double position) {
    // leaderSim.setDistance(position);
    // }

}
