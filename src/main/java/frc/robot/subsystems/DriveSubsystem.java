// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  public final SwerveModule m_frontLeft =
      new SwerveModule(
          "FrontLeft",
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          //DriveConstants.kFrontLeftDriveEncoderPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  public final SwerveModule m_rearLeft =
      new SwerveModule(
        "RearLeft",
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
         // DriveConstants.kRearLeftDriveEncoderPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  public final SwerveModule m_frontRight =
      new SwerveModule(
        "FrontRight",
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          //DriveConstants.kFrontRightDriveEncoderPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  public final SwerveModule m_rearRight =
      new SwerveModule(
        "RearRight",
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          //DriveConstants.kRearRightDriveEncoderPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final AHRS gyro = RobotContainer.getAHRS();
  


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
          

    // objects to send pose info to the dashboard
    private Field2d m_field = new Field2d();
    Pose2d targetPose = new Pose2d(15.513558, 2.748026,new Rotation2d(Units.degreesToRadians(180)));
    Pose2d robotStartPose = new Pose2d(14, 2.75, new Rotation2d(Units.degreesToRadians(270)));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.setAngleAdjustment(90);

    m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          new Rotation2d(getGyro()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()},
            robotStartPose
          );

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    var odom = m_odometry.update(
      new Rotation2d(getGyro()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    SmartDashboard.putString("Odometry Pose", odom.toString());



   // m_field.setRobotPose(m_odometry.getPoseMeters().plus(new Transform2d(new Translation2d(0, 0), new Rotation2d(Units.degreesToRadians(90)))));  // update dashboard
    m_field.setRobotPose(rotatePose2d(m_odometry.getPoseMeters(),90));
    m_field.getObject("Target").setPose(targetPose);
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(getGyro()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 4 * rot, new Rotation2d(getGyro()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    SmartDashboard.putNumber("speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("GyroAngle", gyro.getAngle());
    SmartDashboard.putNumber("GyroPitch", gyro.getPitch());
    SmartDashboard.putNumber("GyroYaw", gyro.getYaw());
    SmartDashboard.putNumber("Odometry X", getPose().getX());
  
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(90);
  }


  public double getGyro() {
    return Math.toRadians(-gyro.getAngle());
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return new Rotation2d(getGyro()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Method to rotate a Pose2d by an angle in degrees
   * 
   * @param pose  The pose to be rotated  x & y are uneffected
   * @param angle The angle in degrees to rotate the pose
   * 
   */

   public Pose2d rotatePose2d(Pose2d pose, double angle){

    double m_poseX = pose.getX();
    double m_poseY = pose.getY();
    Rotation2d m_poseAngle = pose.getRotation();
    Rotation2d m_correctionAngle = new Rotation2d(Units.degreesToRadians(angle));

    return new Pose2d(m_poseX, m_poseY, m_poseAngle.plus(m_correctionAngle));
   }


}
