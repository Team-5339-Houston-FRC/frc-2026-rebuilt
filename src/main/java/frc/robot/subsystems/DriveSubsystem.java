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
import frc.robot.Constants.OperatorConstants;
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
        new SparkBaseMotorChannels(10),
        new SparkBaseMotorChannels(11));

    List<SparkBaseMotorChannels> rightChannels = List.of(
        new SparkBaseMotorChannels(20),
        new SparkBaseMotorChannels(21));

    if (Robot.isSimulation()) {
      m_leftMotor = new SparkMaxMotorArraySim("Drive", leftChannels, Designation.Left, false,
          DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxVoltgage);
      m_rightMotor = new SparkMaxMotorArraySim("Drive", rightChannels, Designation.Right, true,
          DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxVoltgage);
    } else {
      m_leftMotor = new SparkMaxMotorArray("Drive", leftChannels, false, Designation.Left);
      m_rightMotor = new SparkMaxMotorArray("Drive", rightChannels, true, Designation.Right);
    }

    driveTrain = new DifferentialDrive(m_leftMotor.getLeader().motor, m_rightMotor.getLeader().motor);
    driveTrain.setExpiration(.1);
    driveTrain.setMaxOutput(1);
    driveTrain.setDeadband(OperatorConstants.kDriveDeadband);

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    m_drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 7.29, .5, 10, Units.inchesToMeters(3),
        Units.inchesToMeters(21.5), null);

    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftMotor.getDistance(), m_rightMotor.getDistance());
  }

  public void drive(double leftSpeed, double rightSpeed) {
    double leftFeedforward = m_feedforward.calculate(leftSpeed);
    double rightFeedforward = m_feedforward.calculate(rightSpeed);

    // figure out what to do with these values
    // double xSpeedDelivered = leftSpeed *
    // DriveConstants.kMaxSpeedMetersPerSecond;
    // double ySpeedDelivered = rightSpeed *
    // DriveConstants.kMaxSpeedMetersPerSecond;
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        leftSpeed,
        rightSpeed);

    // ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    // m_leftMotor.setSpeeds(wheelSpeeds.leftMetersPerSecond, leftFeedforward);
    // m_rightMotor.setSpeeds(wheelSpeeds.rightMetersPerSecond, rightFeedforward);
    if (Math.abs(leftSpeed) == 0 && Math.abs(rightSpeed) == 0) {
      driveTrain.stopMotor();
      m_rightMotor.setVelocity(0);
      m_leftMotor.setVelocity(0);
    } else {
      driveTrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, true);
    }
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void updateOdometry() {
    double leftDistance = m_leftMotor.getDistance();
    double rightDistance = m_rightMotor.getDistance();

    m_odometry.update(m_drivetrainSim.getHeading(), leftDistance, rightDistance);
    headingPublisher.set(m_drivetrainSim.getHeading());
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
    publisher.set(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrainSim.setInputs(
        m_leftMotor.getVoltage() * RobotController.getInputVoltage(),
        m_rightMotor.getVoltage() * RobotController.getInputVoltage());

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_drivetrainSim.getCurrentDrawAmps()));
    m_drivetrainSim.update(0.02);

    double leftVelocity = m_drivetrainSim.getLeftVelocityMetersPerSecond();
    double rightVelocity = m_drivetrainSim.getRightVelocityMetersPerSecond();

    m_leftMotor.simulationPeriodic(leftVelocity);
    m_rightMotor.simulationPeriodic(rightVelocity);

    updateOdometry();
  }
}
