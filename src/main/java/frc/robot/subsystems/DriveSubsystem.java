// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.abstractions.ISparkMaxMotorArray;
import frc.robot.classes.SparkBaseMotorChannels;
import frc.robot.classes.SparkMaxMotorArray;
import frc.robot.classes.SparkMaxMotorArraySim;
import frc.robot.classes.Designation;

public class DriveSubsystem extends SubsystemBase {

  private final SparkMaxMotorArray m_leftMotor;
  private final SparkMaxMotorArray m_rightMotor;

  private final Field2d m_field = new Field2d();

  private final DifferentialDrivetrainSim m_drivetrainSim;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  PIDController headingController = new PIDController(1.25, 0.0, 1.75);

  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrive driveTrain;
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Robot Pose", Pose2d.struct)
      .publish();

  StructPublisher<Rotation2d> headingPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Heading", Rotation2d.struct)
      .publish();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    List<SparkBaseMotorChannels> leftChannels = List.of(
        new SparkBaseMotorChannels(10, 0),
        new SparkBaseMotorChannels(11, 0));

    List<SparkBaseMotorChannels> rightChannels = List.of(
        new SparkBaseMotorChannels(20, 0),
        new SparkBaseMotorChannels(21, 0));

    if (Robot.isSimulation()) {
      m_leftMotor = new SparkMaxMotorArraySim(leftChannels, Designation.Left, false);
      m_rightMotor = new SparkMaxMotorArraySim(rightChannels, Designation.Right, true);
    } else {
      m_leftMotor = new SparkMaxMotorArray(leftChannels, false, Designation.Left);
      m_rightMotor = new SparkMaxMotorArray(rightChannels, true, Designation.Right);
    }

    driveTrain = new DifferentialDrive(m_leftMotor.getLeader().motor, m_rightMotor.getLeader().motor);

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
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
    double leftFeedforward = m_feedforward.calculate(leftSpeed)
    ;
    double rightFeedforward = m_feedforward.calculate(rightSpeed);

    // figure out what to do with these values
    // double xSpeedDelivered = leftSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    // double ySpeedDelivered = rightSpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftFeedforward, rightFeedforward);
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    // Convert to wheel speeds

    m_leftMotor.setSpeeds(wheelSpeeds.leftMetersPerSecond, leftFeedforward);
     m_rightMotor.setSpeeds(wheelSpeeds.rightMetersPerSecond, rightFeedforward);
    //driveTrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void updateOdometry() {
    double leftDistance = m_leftMotor.getDistance();
    double rightDistance = m_rightMotor.getDistance();

    // m_odometry.update(
    // m_gyro.getRotation2d(), leftDistance, rightDistance);

    m_odometry.update(m_drivetrainSim.getHeading(), leftDistance, rightDistance);
    headingPublisher.set(m_drivetrainSim.getHeading());

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
    // This method will be called once per scheduler run Dduring simulation
    m_drivetrainSim.setInputs(
        m_leftMotor.getVoltage() * RobotController.getInputVoltage(),
        m_rightMotor.getVoltage() * RobotController.getInputVoltage());

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_drivetrainSim.getCurrentDrawAmps()));

    m_drivetrainSim.update(0.02);

    double leftVelocity = m_drivetrainSim.getLeftVelocityMetersPerSecond();
    double leftDistance = m_drivetrainSim.getLeftPositionMeters();

    double rightVelocity = m_drivetrainSim.getRightVelocityMetersPerSecond();
    double rightDistance = m_drivetrainSim.getRightPositionMeters();

    m_leftMotor.simulationPeriodic(leftVelocity, leftDistance);
    m_rightMotor.simulationPeriodic(rightVelocity, rightDistance);

    // updateOdometry();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    // publisher.set(m_odometry.getPoseMeters()); double leftDistance =
    // m_leftMotor.getDistance();

    // m_odometry.update(
    // m_gyro.getRotation2d(), leftDistance, rightDistance);

    m_odometry.update(
        m_drivetrainSim.getHeading(), leftDistance, rightDistance);
  }
}
