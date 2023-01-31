// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCalibrateCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems


  // The driver's controller
  private static Joystick joystick1 = new Joystick(0);
  private static Joystick joystick2 = new Joystick(1);
  private static GenericHID gamePad1 = new GenericHID(2);
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private static JoystickButton joystick2Button9 = new JoystickButton(joystick2, 8);

    public void resetGyro() {
      m_robotDrive.zeroHeading();
    }

    public void drive() {
      //Joystick values
      
      double x = -joystick1.getY();
      double y = -joystick1.getX();
      double rot = -joystick2.getX();
      double deadband = 0.2;
      if(x > -deadband && x < deadband) {
        x = 0;
      }
      if(y > -deadband && y < deadband) {
        y = 0;
      }
      if(rot > -deadband && rot < deadband) {
        rot = 0;
      }

      x = Math.pow(x, 3);
      y = Math.pow(y, 3);
      rot = Math.pow(rot, 3);

      /*  Next step should be to convert getVision / getPose from returning translation from camera to the April Tag to
       *  the actual field position
       *  then here we can define a desired filed location
       *  and calculate the translation from current position to the desired and auto travel that path
       * 
       *  Future step will be to use the photon vision library to merge the april tag location with the swerve obometry position
       */

      if(joystick2Button9.getAsBoolean()) {

        double m_autoScaling = 2;
        double m_auto_Maximum = 0.3;

        rot = 0;
        Pose2d pose = getVision();
        x = pose.getX() * m_autoScaling;
        y = pose.getY() * m_autoScaling;
        

        double test = Math.max(Math.abs(x),Math.abs(y));
        if(test > m_auto_Maximum){
          x = x * m_auto_Maximum / test;
          y = y * m_auto_Maximum / test;
        }

        /* old code here
        if(x > 0.1) {
          x = 0.1; 
        } else if (x < -0.1) {
          x = -0.1;
        }

 
        if(y > 0.1) {
          y = 0.1;
        } else if (y < -0.1) {
          y = -0.1;
        }

        */


        rot = pose.getRotation().getDegrees() / (-180 * 3);

      }

      m_robotDrive.drive(x, y, rot, true);
    }

    //gets target position
    public Pose2d getVision() {
      SmartDashboard.putString("TargetPose", m_visionSubsystem.getPose().toString());
      Pose2d fieldPose = m_visionSubsystem.getPose();
      Pose2d targetPose = new Pose2d(14, 2.75, new Rotation2d(0));
      Transform2d robotToTarget = targetPose.minus(fieldPose);


      double x = fieldPose.getX();
      double y = fieldPose.getY();
      double deadband = 0.2;
      Pose2d returnPose = new Pose2d(0, 0, robotToTarget.getRotation());
      
      if(x > deadband || x < -deadband) {
        returnPose = new Pose2d(robotToTarget.getX(), 0, robotToTarget.getRotation());
      } 
      if(y > deadband || y < -deadband) {
        returnPose = new Pose2d(returnPose.getX(), robotToTarget.getY(), robotToTarget.getRotation());
      }
      return returnPose;
      //return m_visionCommand;
    }

    public void resetDriveEncoders() {
        m_robotDrive.resetEncoders();
    }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    //m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        /*new RunCommand(
            () ->
                m_robotDrive.drive(
                    joystick1.getY(),
                    joystick1.getX(),
                    joystick2.getX(),
                    true),
            m_robotDrive));*/

  }

  public void Calibrate() {
    SmartDashboard.putData("Calibrate", new DriveCalibrateCommand(m_robotDrive));
  }
  
  
  
  
  
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();


  private final VisionCommand m_visionCommand = new VisionCommand(m_visionSubsystem);


  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();



  public static double getGamepad1Axis0() {
    
    double axis = gamePad1.getRawAxis(0);
    //if(axis > -0.1 && axis < 0.1) {
      //axis = 0;
    //}
    return axis;
  }
  private final GripperCommand m_gripperCommand = new GripperCommand(m_gripperSubsystem, RobotContainer::getGamepad1Axis0);

  public Command getGripperCommand() {
    return m_gripperCommand;
  }







  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d(0)));//exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  public static AHRS getAHRS() {
    //Starts the IMU
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      AHRS ahrs = new AHRS(SPI.Port.kMXP);
      // ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
      return ahrs;
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      return null;
    }
}
}
