package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

public class SparkMaxMotorArraySim extends SparkMaxMotorArray {

    private final double maxSpeed;
    private SparkMAXMotorSim leaderSim;
    private final List<SparkMAXMotorSim> motorSims = new ArrayList<SparkMAXMotorSim>();

    public SparkMaxMotorArraySim(String subsystem, List<SparkBaseMotorChannels> channels, Designation designation,
            boolean isInverted, double maxSpeed, double maxVoltage) {
        super(subsystem, channels, isInverted, designation);

        this.maxSpeed = maxSpeed;
        for (SparkMAXMotor motor : this.motors) {
            if (leaderSim == null) {
                leaderSim = new SparkMAXMotorSim(motor, maxVoltage, maxSpeed);
                motorSims.add(leaderSim);
            } else {
                SparkMAXMotorSim follower = new SparkMAXMotorSim(motor, maxVoltage, maxSpeed);
                motorSims.add(follower);
            }
        }
    }

    @Override
    public void simulationPeriodic(double velocity, double distance) {
        double rotationsPerSecond = this.maxSpeed / 60.0;
        double deltaTime = 0.02; // 20ms loop
        double deltaRotations = rotationsPerSecond * deltaTime;

        SparkMAXMotor leader = motors.get(0);
        setDistance(distance);
        leaderSim.simulationPeriodic(velocity, 12, deltaRotations);
    }

    @Override
    public void setSpeeds(double metersPerSecond, double feedforward) {
        super.setSpeeds(metersPerSecond, feedforward);
        //SparkMAXMotor leader = motors.get(0);
        //leader.setSpeeds(metersPerSecond, feedforward);
    }

}
