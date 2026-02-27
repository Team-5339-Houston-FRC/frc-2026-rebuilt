package frc.robot.abstractions;

public interface ISparkMaxMotor {

    void setVelocity(double velocity);

    double getVelocity();

    void simulationPeriodic(double velocity, double vbus, double dt);

    double getAppliedOutput();
}
