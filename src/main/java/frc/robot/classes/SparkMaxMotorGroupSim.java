package frc.robot.classes;

import frc.robot.abstractions.ISparkMaxMotorGroup;

public class SparkMaxMotorGroupSim implements ISparkMaxMotorGroup {

    private final SparkMAXMotorSim _leader;
    private final SparkMAXMotorSim _follower;

    public SparkMaxMotorGroupSim(int channelA, int channelB, int channelC, int channelD, boolean isInverted) {
        _leader = SparkMAXMotorSim.CreateSparkMaxMotor(channelA, channelB, isInverted);
        _follower = SparkMAXMotorSim.CreateSparkMaxMotor(channelC, channelD, isInverted);
    }

    public void setSpeeds(double metersPerSecond, double feedforward) {
        _leader.setSpeeds(metersPerSecond, feedforward);
    }

    // set the motor's speed
    public void setMotorSpeed(double speed) {
        _leader.setMotorSpeed(speed);
    }

    public double getDistance() {
        return _leader.getDistance();
    }

    public double get() {
        return _leader.get();
    }
    
    public void setDistance(double position) {
        _leader.setDistance(position);
    }

    public void setRate(double rate) {
        _leader.setRate(rate);
    }
}
