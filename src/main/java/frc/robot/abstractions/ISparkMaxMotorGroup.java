package frc.robot.abstractions;

public interface ISparkMaxMotorGroup {

    void setSpeeds(double metersPerSecond, double feedforward);

    // set the motor's speed
    void setMotorSpeed(double speed);

    double getDistance();

    double get();

    void setDistance(double position);

    void setRate(double rate);
}