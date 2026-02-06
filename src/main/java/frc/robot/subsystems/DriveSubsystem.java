// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.SparkMAXMotor;

public class DriveSubsystem extends SubsystemBase {
  private final SparkMAXMotor m_leftMotor;
  private final SparkMAXMotor m_rightMotor;

  private final AHRS m_gyro = new AHRS(null);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftMotor = new SparkMAXMotor(0, 1, false);
    m_rightMotor = new SparkMAXMotor(0, 1, true);
    
  m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(), m_leftMotor.getDistance(), m_rightMotor.getDistance());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    m_leftMotor.setSpeeds(speeds.leftMetersPerSecond, leftFeedforward);
    m_rightMotor.setSpeeds(speeds.rightMetersPerSecond, rightFeedforward);
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_leftMotor.setMotorSpeed(leftSpeed);
    m_rightMotor.setMotorSpeed(rightSpeed);
  }

  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftMotor.getDistance(), m_rightMotor.getDistance());
  }
}
