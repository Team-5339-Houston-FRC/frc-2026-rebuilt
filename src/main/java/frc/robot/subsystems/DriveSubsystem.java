// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.abstractions.ISparkMaxMotorGroup;
import frc.robot.classes.SparkMaxMotorGroup;
import frc.robot.classes.SparkMaxMotorGroupSim;

public class DriveSubsystem extends SubsystemBase {
  private final ISparkMaxMotorGroup m_leftMotor;
  private final ISparkMaxMotorGroup m_rightMotor;

  private final Field2d m_field = new Field2d();

  private final DifferentialDrivetrainSim m_drivetrainSim;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Robot Pose", Pose2d.struct)
      .publish();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    if (Robot.isSimulation()) {
      m_leftMotor = new SparkMaxMotorGroupSim(0, 1, 2, 3, false);
      m_rightMotor = new SparkMaxMotorGroupSim(4, 5, 6, 7, true);
    } else {
      m_leftMotor = new SparkMaxMotorGroup(0, 1, 2, 3, false);
      m_rightMotor = new SparkMaxMotorGroup(4, 5, 6, 7, true);
    }
    ;

    m_drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 15, .5, 1, Units.inchesToMeters(2),
        Units.inchesToMeters(4), null);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
    publisher.set(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_drivetrainSim.setInputs(
        m_leftMotor.get() * RobotController.getInputVoltage(), m_rightMotor.get() * RobotController.getInputVoltage());
    m_drivetrainSim.update(0.02);
    m_leftMotor.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftMotor.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightMotor.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightMotor.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
    publisher.set(m_odometry.getPoseMeters());
  }
}
