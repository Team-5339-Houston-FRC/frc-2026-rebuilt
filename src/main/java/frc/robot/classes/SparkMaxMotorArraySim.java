package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

public class SparkMaxMotorArraySim extends SparkMaxMotorArray {

    private SparkMAXMotorSim leaderSim;
    private final List<SparkMAXMotorSim> motorSims = new ArrayList<SparkMAXMotorSim>();

    public SparkMaxMotorArraySim(String subsystem, List<SparkBaseMotorChannels> channels, Designation designation,
            boolean isInverted, double maxSpeed, double maxVoltage) {
        super(subsystem, channels, isInverted, designation, maxSpeed, maxVoltage);

        for (SparkMAXMotor motor : this.motors) {
            if (leaderSim == null) {
                leaderSim = new SparkMAXMotorSim(subsystem, motor, maxVoltage, maxSpeed);
                motorSims.add(leaderSim);
            } else {
                SparkMAXMotorSim follower = new SparkMAXMotorSim(subsystem, motor, maxVoltage, maxSpeed);
                motorSims.add(follower);
            }
        }
    }

    @Override
    public void simulationPeriodic(double velocity) {
        leaderSim.simulationPeriodic(velocity);
    }

    public void setVelocity(double velocity) {
        leaderSim.setVelocity(velocity);
    }
}
