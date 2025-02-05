// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CowboyUtils;
import frc.robot.Constants.DriveTrainConstants;

public class DriveSubsystem extends SubsystemBase {
  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  private final XRPGyro m_gyro = new XRPGyro();

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      CowboyUtils.inchesToMeters(DriveTrainConstants.TRACK_WIDTH_INCH));

  private final DifferentialDriveOdometry odometry;
  private final Field2d m_field2d = new Field2d();
  SysIdRoutine routine;

  /** Creates a new XRPDrivetrain. */
  public DriveSubsystem() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(
        (Math.PI * DriveTrainConstants.WHEEL_DIAMETER_INCH) / DriveTrainConstants.COUNTS_PER_REVOLUTION);
    m_rightEncoder.setDistancePerPulse(
        (Math.PI * DriveTrainConstants.WHEEL_DIAMETER_INCH) / DriveTrainConstants.COUNTS_PER_REVOLUTION);
    resetEncoders();
    m_gyro.reset();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    odometry = new DifferentialDriveOdometry(getGyroAngleRotation2d(),
        CowboyUtils.inchesToMeters(getLeftDistanceInch()),
        CowboyUtils.inchesToMeters(getRightDistanceInch()));

    AutoBuilder.configureRamsete(this::getRobotPose, this::resetOdometry, this::getChassisSpeeds,
        this::driveChassisSpeeds, 1, .6,
        new ReplanningConfig(false, false),
        () -> false, this);

  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDriveSpeedsMetersPerSec(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public void arcadeDriveSpeedsMetersPerSec(double x, double rot) {
    // Convert from meters per second to percentage of max speed (-1.0 to 1.0)
    double xPercent = x / DriveTrainConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotPercent = rot / DriveTrainConstants.MAX_SPEED_METERS_PER_SECOND;

    // Ensure the values stay within the range of -1.0 to 1.0
    xPercent = Math.max(Math.min(xPercent, 1.0), -1.0);
    rotPercent = Math.max(Math.min(rotPercent, 1.0), -1.0);

    // Use arcadeDrive to apply the converted percentages
    m_diffDrive.arcadeDrive(xPercent, rotPercent);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getGyroAngleDegrees() {
    return m_gyro.getAngleZ();
  }

  public Rotation2d getGyroAngleRotation2d() {
    return new Rotation2d(Math.toRadians(getGyroAngleDegrees()));
  }

  private void updateOdometry() {
    odometry.update(getGyroAngleRotation2d(),
        getWheelPositionsMeters());
    m_field2d.setRobotPose(getRobotPose());
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroAngleRotation2d(), getWheelPositionsMeters(), pose);
  }

  public DifferentialDriveWheelPositions getWheelPositionsMeters() {
    return new DifferentialDriveWheelPositions(CowboyUtils.inchesToMeters(getLeftDistanceInch()),
        CowboyUtils.inchesToMeters(getRightDistanceInch()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run, every 20 ms.
    updateOdometry();
    m_diffDrive.feed();
    SmartDashboard.putNumber("Gyro Angle", getGyroAngleDegrees());
    SmartDashboard.putData(m_field2d);

  }

  public Command resetGyroAngle() {
    return new StartEndCommand(() -> {
      resetGyro();
    }, () -> {
    });
  }

}
