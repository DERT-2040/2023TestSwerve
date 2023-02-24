// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

// COMMANDS  //
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRetractCommand;
import frc.robot.commands.ArmNegCommand;
//import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCalibrateCommand;
import frc.robot.commands.GripperReleaseCommand;
import frc.robot.commands.IntakeExhaleCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeInhaleCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.CargoRequestCommand;

// SUBSYSTEMS  //
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeInhaleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PDHMonitor;
import frc.robot.subsystems.VisionSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


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

  public void init() {
    m_LedSubsystem.setColor(true);
  }

  public void periodic() {
    checkButtonInputs();
    m_PDHMonitor.periodic();
  }



  // The driver's controller
  // define Joysticks and GamePads

  private static Joystick joystick1 = new Joystick(0);
  private static Joystick joystick2 = new Joystick(1);
  private static GenericHID gamePad1 = new GenericHID(2);

  //  define buttons and controls

  private static JoystickButton joystick1Button2 = new JoystickButton(joystick1, 2);
  private static JoystickButton joystick1Button3 = new JoystickButton(joystick1, 3);
  private static JoystickButton joystick2Button8 = new JoystickButton(joystick2, 8);
  private static JoystickButton joystick2Button2 = new JoystickButton(joystick2, 2);
  private static JoystickButton joystick2Button3 = new JoystickButton(joystick2, 3);
  private static JoystickButton joystick2Button4 = new JoystickButton(joystick2, 4);
  private static JoystickButton joystick2Button5 = new JoystickButton(joystick2, 5);
  private static JoystickButton joystick1Button11 = new JoystickButton(joystick1, 11);
  private static JoystickButton joystick1Button10 = new JoystickButton(joystick1, 10);

  private static JoystickButton gamePad1Button3  = new JoystickButton(gamePad1,  3); 


  /*  ****          Define The robot's subsystems       ****   /
  //
  //  each subsystem is defined in a file located in the subsystems folder
  //  here we instantiate the subsystem
  */

  private final DriveSubsystem        m_robotDrive =              new DriveSubsystem();
  private final VisionSubsystem       m_visionSubsystem =         new VisionSubsystem();
  public  final LEDSubsystem          m_LedSubsystem =            new LEDSubsystem();
  private final GripperSubsystem      m_gripperSubsystem =        new GripperSubsystem();
  private final PDHMonitor            m_PDHMonitor =              new PDHMonitor();
  private final ArmSubsystem          m_armSubsystem =            new ArmSubsystem();
  private final IntakeExtendSubsystem m_intakeExtendSubsystem =   new IntakeExtendSubsystem();
  private final IntakeInhaleSubsystem m_intakeInhaleSubsystem =   new IntakeInhaleSubsystem();
  

  /*  ****          Define The robot's Commands       ****   /
  //
  //  each command is defined in a file located in the commands folder
  //  here we instantiate the commands
  //  commands are used further down in this file to accomplish various tasks
  //  usally associated with an input defined above
  */

  private final VisionCommand         m_visionCommand =         new VisionCommand(m_visionSubsystem);
  private final ArmCommand            m_armCommand =            new ArmCommand(m_armSubsystem);
  private final ArmNegCommand         m_armNegCommand =         new ArmNegCommand(m_armSubsystem);
  private final IntakeExtendCommand   m_intakeExtendCommand =   new IntakeExtendCommand(m_intakeExtendSubsystem);
  private final IntakeRetractCommand  m_intakeRetractCommand =  new IntakeRetractCommand(m_intakeExtendSubsystem);
  private final IntakeInhaleCommand   m_intakeInhaleCommand =   new IntakeInhaleCommand(m_intakeInhaleSubsystem);
  private final IntakeExhaleCommand   m_intakeExhaleCommand =   new IntakeExhaleCommand(m_intakeInhaleSubsystem);
  private final GripperReleaseCommand m_gripperReleaseCommand = new GripperReleaseCommand(m_gripperSubsystem, RobotContainer::getGamepad1Axis0);
  private final CargoRequestCommand   m_cargoRequestCommand =   new CargoRequestCommand(m_LedSubsystem);
  private final ArmExtendCommand      m_armExtendCommand    =   new ArmExtendCommand(m_armSubsystem);
  private final ArmRetractCommand     m_armRetractCommand   =   new ArmRetractCommand(m_armSubsystem);
  

  public Command getGripperCommand() {
    return m_gripperReleaseCommand;
  }




    public void checkButtonInputs() {
      //sets LEDs to Purple
     /*  if(gamePad1.getRawButton(3)) {
        m_LedSubsystem.setColor(false);
      }*/
      gamePad1Button3.whileTrue(m_cargoRequestCommand);

      //sets LEDs to Yellow
      if(gamePad1.getRawButton(4)) {
        m_LedSubsystem.setColor(true);
      }

      joystick1Button2.whileTrue(m_armCommand);
      joystick1Button3.whileTrue(m_armNegCommand);
      joystick2Button4.whileTrue(m_intakeExhaleCommand);
      joystick2Button5.whileTrue(m_intakeInhaleCommand);
      joystick2Button3.whileTrue(m_intakeExtendCommand);
      joystick2Button2.whileTrue(m_intakeRetractCommand);
      joystick1Button11.whileTrue(m_armExtendCommand);
      joystick1Button10.whileTrue(m_armRetractCommand);
      
    }


    
    double filteredX = 0;
    double filteredY = 0;
    

    public void drive() {
      
      double executionTime = Timer.getFPGATimestamp();
      

    //Joystick values
      double x = -joystick1.getX();
      double y = joystick1.getY();
      double rot = joystick2.getX();
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

      if(joystick2Button8.getAsBoolean()) {

      
        double m_maximum = 1;

        Pose2d pose = getVision();
        x = pose.getX();
        y = pose.getY();
        

        double test = Math.max(Math.abs(x),Math.abs(y));
        if(test > m_maximum){
          x = x * m_maximum / test;
          y = y * m_maximum / test;
        }

        double filter = 0.25;
        filteredX = (x - filteredX) * filter + filteredX;
        x = filteredX;
        filteredY = (y - filteredY) * filter + filteredY;
        y = filteredY;
        
        
        
        rot = pose.getRotation().getDegrees() / (180);
        
        if(rot > .5) {
          rot = .5;
        } else if(rot < -.5) {
          rot = -.5;
        }
      }

      double speed = 1;//(-joystick1.getZ() + 1) / 2;
      
      if(RobotController.getBatteryVoltage() < 10) {
        speed = 0;
      }

      m_robotDrive.drive(speed * x, speed * y, speed * rot, true);
      SmartDashboard.putNumber("Drive Execution Time", Timer.getFPGATimestamp() - executionTime);
    }


    // Target Pose is the desired location on the field to drive to
      Pose2d targetPose = new Pose2d(14, 2.75, new Rotation2d(0));
      
      PIDController m_xControl = new PIDController(2,0.2,0);
      PIDController m_yControl = new PIDController(1,0.2,0);



    //gets target position  preform drive to the targeted postion on the field when a button is pushed

    public Pose2d getVision() {

      //SmartDashboard.putString("robotDrivePose ", m_robotDrive.getPose().toString());
      Pose2d odometryPose = new Pose2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY() /*+ targetPose.getY() * 2*/, m_robotDrive.getPose().getRotation());
      //SmartDashboard.putString("Odometry Pose", odometryPose.toString());
      //SmartDashboard.putString("Raw Odom Pose", m_robotDrive.getPose().toString());
      Pose2d visionPose = m_visionSubsystem.getPose();
      //SmartDashboard.putString("Vision Pose", visionPose.toString());


      Pose2d fieldPose = odometryPose;
      m_xControl.setIntegratorRange(-0.1,0.1);
      m_yControl.setIntegratorRange(-0.1,0.1);


      if(visionPose.getX() != -999){
        fieldPose = new Pose2d((odometryPose.getX() + visionPose.getX()) / 2, (odometryPose.getY() + visionPose.getY()) / 2, odometryPose.getRotation());
        

      }
      

      Pose2d robotToTarget = new Pose2d(targetPose.getX() - fieldPose.getX(), targetPose.getY() - fieldPose.getY(), new Rotation2d(-targetPose.getRotation().getRadians() + fieldPose.getRotation().getRadians()));


      

      double x = robotToTarget.getX();
      double y = robotToTarget.getY();
      Rotation2d rot = robotToTarget.getRotation();

      Pose2d returnPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));


  

        double m_xError = Math.abs(targetPose.getX() - fieldPose.getX());
        double m_yError = Math.abs(targetPose.getY() - fieldPose.getY());

        SmartDashboard.putNumber("x error",m_xError);
        SmartDashboard.putNumber("y error",m_yError);

        if(m_xError > 0.1016/2){  // only update drive if error is more than 2 inches
          x = m_xControl.calculate(fieldPose.getX(), targetPose.getX());
        } else{
          x = 0;
        }

        if(m_yError > 0.1016/2){ // only update drive if error is more than 2 in
          y = m_yControl.calculate(fieldPose.getY(), targetPose.getY());
        } else {
          y = 0;
        }
          rot = robotToTarget.getRotation();
        

        returnPose = new Pose2d(y, -x, rot);// robotToTarget.getY(), robotToTarget.getRotation());




      return returnPose;

    }

    public void resetDriveEncoders() {
        m_robotDrive.resetEncoders();
    }


  public void Calibrate() {
    SmartDashboard.putData("Calibrate", new DriveCalibrateCommand(m_robotDrive));
  }
  
  
  
  
  
  


  public void LEDIdle() {
    m_LedSubsystem.idlePattern();
  }

  public void LEDVoltage() {
    m_LedSubsystem.lowVoltage();
  }



  public static double getGamepad1Axis0() {
    
    double axis = gamePad1.getRawAxis(0);
    //if(axis > -0.1 && axis < 0.1) {
      //axis = 0;
    //}
    return axis;
  }
  






  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

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

public void resetGyro() {
  m_robotDrive.zeroHeading();
}

}
