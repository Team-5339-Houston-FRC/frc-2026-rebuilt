package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class SparkMaxMotorArraySim extends SparkMaxMotorArray {

    private final Designation designation;
    private SparkMAXMotorSim leaderSim;
    private final List<SparkMAXMotorSim> motorSims = new ArrayList<SparkMAXMotorSim>();

    public SparkMaxMotorArraySim(List<SparkBaseMotorChannels> channels, Designation designation, boolean isInverted) {
        this.designation = designation;
        for (SparkBaseMotorChannels channel : channels) {
            if (leaderSim == null) {
                leaderSim = SparkMAXMotorSim.CreateSparkMAXMotorSim(channel, isInverted);
            } else {
                SparkMAXMotorSim follower = SparkMAXMotorSim.CreateSparkMAXMotorSim(channel, isInverted);
                motorSims.add(follower);
            }
        }
    }

    public void setSpeeds(double metersPerSecond, double feedforward) {
        leaderSim.setSpeeds(metersPerSecond, feedforward);
        leaderSim.setVelocity(metersPerSecond);
    }

    @Override
    public void simulationPeriodic(DifferentialDrivetrainSim drivetrainSim, Designation designation) {
        double velocity = 0;
        double rotationsPerSecond = 1500 / 60.0;
        double deltaTime = 0.02; // 20ms loop
        double deltaRotations = rotationsPerSecond * deltaTime;

        if (designation == Designation.Left && this.designation == Designation.Left) {
            velocity = drivetrainSim.getLeftVelocityMetersPerSecond();
            setDistance(drivetrainSim.getLeftPositionMeters());
            leaderSim.simulationPeriodic(velocity, deltaTime, deltaRotations);
        } else if (designation == Designation.Right && this.designation == Designation.Right) {
            velocity = drivetrainSim.getRightVelocityMetersPerSecond();
            setDistance(drivetrainSim.getRightPositionMeters());
            leaderSim.simulationPeriodic(velocity, deltaTime, deltaRotations);
        }
    }

    @Override
    public void setVelocity(double speed) {
        leaderSim.setVelocity(speed);
    }

    @Override
    public double getDistance() {
        return leaderSim.getDistance();
    }

    @Override
    public double get() {
        return leaderSim.get();
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
