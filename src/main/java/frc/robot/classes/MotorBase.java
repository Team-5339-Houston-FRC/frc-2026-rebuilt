package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public abstract class MotorBase {

    private final PWMMotorController _leader;
    private final PWMMotorController _follower;
    private final Encoder _encoder;
    private final PIDController _pidController;

    public MotorBase(int channelA, int channelB, boolean isInverted) {
        _pidController = new PIDController(0.1, 0.0, 0.0);

        _leader = CreateMotor(channelA, isInverted);
        _follower = CreateMotor(channelB, isInverted);

        _leader.addFollower(_follower);

        _encoder = new Encoder(channelA, channelB);
        _leader.setInverted(isInverted);
        _encoder.setDistancePerPulse(0.01); // Example distance per pulse
    }

    protected abstract PWMMotorController CreateMotor(int channel, boolean isInverted);


    public void setSpeeds(double metersPerSecond, double feedforward) {
        double output = _pidController.calculate(_encoder.getRate(), metersPerSecond);

        _leader.setVoltage(output + feedforward);
    }

    // set the motor's speed
    public void setMotorSpeed(double speed) {
        _leader.set(speed);
    }

    public double getDistance() {
        return _encoder.getDistance();
    }
}
