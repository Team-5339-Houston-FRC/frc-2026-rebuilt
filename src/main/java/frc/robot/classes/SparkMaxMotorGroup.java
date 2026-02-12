package frc.robot.classes;

import com.revrobotics.spark.SparkMax;

import frc.robot.abstractions.ISparkMaxMotorGroup;

public class SparkMaxMotorGroup implements ISparkMaxMotorGroup {

    private final SparkMAXMotor _leader;
    private final SparkMAXMotor _follower;

    public SparkMaxMotorGroup(int channelA, int channelB, int channelC, int channelD, boolean isInverted) {
        _leader = new SparkMAXMotor(channelA, channelB, isInverted);

        SparkBaseMotorConfig<SparkMax> followerConfig = new SparkBaseMotorConfig<SparkMax>(
                new SparkBaseMotorChannels(channelC, channelD), true, _leader);

        _follower = new SparkMAXMotor(followerConfig);
    }

    @Override
    public void setSpeeds(double metersPerSecond, double feedforward) {
        _leader.setSpeeds(metersPerSecond, feedforward);
    }

    // set the motor's speed
    @Override
    public void setMotorSpeed(double speed) {
        _leader.setVelocity(speed);
    }

    @Override
    public double getDistance() {
        return _leader.getDistance();
    }

    public double get() {
        return _leader.get();
    }

    public void setDistance(double position) {
    }

    public void setRate(double rate) {

    }
}
