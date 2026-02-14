package frc.robot.abstractions;

import frc.robot.classes.SparkMAXMotor;

public interface ISparkMaxMotorArray {

    // void setSpeeds(double metersPerSecond, double feedforward);

    // // set the motor's speed
    // void setMotorSpeed(double speed);

     double getDistance();
     double getVoltage();
     SparkMAXMotor getLeader();

    // double get();

    // void setDistance(double position);

    // void setRate(double rate);
}