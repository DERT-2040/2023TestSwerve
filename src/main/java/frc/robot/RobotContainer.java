// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

// COMMANDS  //
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmRetractCommand;
import frc.robot.commands.ArmSelectedPositionCommand;
import frc.robot.commands.ArmNegCommand;
import frc.robot.commands.DriveCalibrateCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GripperConeCommand;
import frc.robot.commands.GripperCubeCommand;
import frc.robot.commands.GripperReleaseCommand;
import frc.robot.commands.IntakeExtendCommand;
import frc.robot.commands.IntakeInhaleCommand;
import frc.robot.commands.TurntableLeftCommand;
import frc.robot.commands.TurntableRightCommand;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.CargoRequestCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// SUBSYSTEMS  //
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeInhaleSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PDHMonitor;
import frc.robot.subsystems.SuperVisoryDrive;
import frc.robot.subsystems.TurntableSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GyroScope;

import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings();

  }

  int last_pov;

  public void init() {
    m_LedSubsystem.setColor(true);
    last_pov = gamePad1.getPOV();
  }

  public void periodic() {
    checkButtonInputs();

    m_PDHMonitor.periodic();

    if(!m_driveCommand.isScheduled()) {   //  Make sure the drive command is scheduled
      m_driveCommand.schedule();
    }



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
  private static JoystickButton joystick1Button6 = new JoystickButton(joystick1, 6);
  private static JoystickButton joystick1Button7 = new JoystickButton(joystick1, 7);
  public static  JoystickButton joystick2Button9 = new JoystickButton(joystick2, 9);

  //blue
  private static JoystickButton gamePad1Button3  = new JoystickButton(gamePad1,  3); 
  //green
  private static JoystickButton gamePad1Button1  = new JoystickButton(gamePad1, 1);
  //red
  private static JoystickButton gamePad1Button2  = new JoystickButton(gamePad1, 2);
  //yellow
  private static JoystickButton gamePad1Button4  = new JoystickButton(gamePad1, 4);
  //start button
  private static JoystickButton gamePad1Button8  = new JoystickButton(gamePad1, 8);
  //Right Joystick press
  private static JoystickButton gamePad1Button10  = new JoystickButton(gamePad1, 10);
  
  
  private static POVButton      gamePad1POVUp =         new POVButton(gamePad1, 0);
  private static POVButton      gamePad1POVUpRight =    new POVButton(gamePad1, 45);
  private static POVButton      gamePad1POVRight =      new POVButton(gamePad1, 90);
  private static POVButton      gamePad1POVDownRight =  new POVButton(gamePad1, 135);
  private static POVButton      gamePad1POVDown =       new POVButton(gamePad1, 180);
  private static POVButton      gamePad1POVDownLeft =   new POVButton(gamePad1, 225);
  private static POVButton      gamePad1POVLeft =       new POVButton(gamePad1, 270);
  private static POVButton      gamePad1POVUpLeft =     new POVButton(gamePad1, 315);

  private double getRightY() {
    return Math.pow(gamePad1.getRawAxis(5), 3);
    
  }
  private double getRightX() {
    return -Math.pow(gamePad1.getRawAxis(4), 3);
  }


  /*  ****          Define The robot's subsystems       ****   /
  //
  //  each subsystem is defined in a file located in the subsystems folder
  //  here we instantiate the subsystem
  */

  public static final DriveSubsystem  m_robotDrive             =  new DriveSubsystem();
  public static final SuperVisoryDrive m_drive                 =  new SuperVisoryDrive();
  public static final VisionSubsystem m_visionSubsystem        =  new VisionSubsystem();
  public  final LEDSubsystem          m_LedSubsystem           =  new LEDSubsystem();
  private final PDHMonitor            m_PDHMonitor             =  new PDHMonitor();
  private final ArmSubsystem          m_armSubsystem           =  new ArmSubsystem();
  private final IntakeExtendSubsystem m_intakeExtendSubsystem  =  new IntakeExtendSubsystem();
  private final IntakeInhaleSubsystem m_intakeInhaleSubsystem  =  new IntakeInhaleSubsystem();
  private final TurntableSubsystem    m_TurntableSubsystem     =  new TurntableSubsystem();
  public final  GyroScope             m_Gyro                   =  new GyroScope();

  /*  ****          Define The robot's Commands       ****   /
  //
  //  each command is defined in a file located in the commands folder
  //  here we instantiate the commands
  //  commands are used further down in this file to accomplish various tasks
  //  usally associated with an input defined above
  */
  private final DriveCommand          m_driveCommand          = new DriveCommand(m_drive, 
        -joystick1.getX(),              // pass driver joystick x request
        joystick1.getY(),               // pass driver joystick y request
        joystick2.getX(),               // pass driver rotate request
        joystick2Button8.getAsBoolean() // pass auto drive  command (enables robot to drive to selected location)
        );
  private final VisionCommand         m_visionCommand =         new VisionCommand(m_visionSubsystem);
  private final ArmCommand            m_armCommand =            new ArmCommand(m_armSubsystem);
  private final ArmNegCommand         m_armNegCommand =         new ArmNegCommand(m_armSubsystem);
  private final ArmManualCommand      m_armManualCommand =      new ArmManualCommand(m_armSubsystem, this::getRightY, this::getRightX, this::getArmManualMode);
  private final ArmSelectedPositionCommand m_armSelectedPositionCommand =   new ArmSelectedPositionCommand(m_armSubsystem, this::getArmPositionSetting);
  private final IntakeExtendCommand   m_intakePositionCommand = new IntakeExtendCommand(m_intakeExtendSubsystem, 0);
  private final GripperReleaseCommand m_gripperReleaseCommand = new GripperReleaseCommand(m_armSubsystem);
  private final GripperCubeCommand m_gripperCubeCommand = new GripperCubeCommand(m_armSubsystem);
  private final GripperConeCommand    m_gripperConeCommand =    new GripperConeCommand(m_armSubsystem);
  private final CargoRequestCommand   m_cargoRequestCommand =   new CargoRequestCommand(m_LedSubsystem);
  private final ArmExtendCommand      m_armExtendCommand    =   new ArmExtendCommand(m_armSubsystem);
  private final ArmRetractCommand     m_armRetractCommand   =   new ArmRetractCommand(m_armSubsystem);
  private final TurntableRightCommand m_TurntableRightCommand = new TurntableRightCommand(m_TurntableSubsystem);
  private final TurntableLeftCommand  m_TurntableLeftCommand =  new TurntableLeftCommand(m_TurntableSubsystem);
  private final IntakeInhaleCommand    m_intakeConeCommand   =  new IntakeInhaleCommand(m_intakeInhaleSubsystem, 1);
  private final IntakeInhaleCommand    m_intakeCubeCommand   =  new IntakeInhaleCommand(m_intakeInhaleSubsystem, 2);
  private final IntakeInhaleCommand    m_intakeReverseCommand=  new IntakeInhaleCommand(m_intakeInhaleSubsystem, 3);

  //0 is low, 1 is mid, and 2 is high
  private int armPositionSetting = 0;
  //private int armExtensionSetting = 0;

  public int getArmPositionSetting() {
    return armPositionSetting;
  }

  private boolean armManualMode = false;
  public boolean getArmManualMode() {
    return armManualMode;
  }

  public Pose2d getVision(){  /// this is redundant   we have an issue with Robot calling this and SuperVisoryDrive as well

    return(m_drive.getVision());  // BUT need to make sure getVision is called every loop

  }


    public int CheckAlliance() {
      //Blue Alliance = 1
      //Red Alliance = 2
      DriverStation.Alliance alliance;
      alliance = DriverStation.getAlliance();
      if (alliance == Alliance.Blue) {
        return 1;
      } else if (alliance == Alliance.Red) {
        return 2;
      } else {
        return 0;
      }
    }

    public void checkButtonInputs() {  

      SmartDashboard.putNumber("Allience Number (Blue 1, Red 2)", CheckAlliance());
      //sets LEDs to Purple
     /*  if(gamePad1.getRawButton(3)) {
        m_LedSubsystem.setColor(false);
      }*/
      gamePad1Button2.whileTrue(m_cargoRequestCommand);


      if (last_pov != gamePad1.getPOV()) {
        if (gamePad1.getPOV() == 0) {
          if (armPositionSetting != 2) {
          armPositionSetting += 1;
          } else {
            armPositionSetting = 0;
          }
        } else if (gamePad1.getPOV() == 180) {
          if (armPositionSetting != 0) {
          armPositionSetting -= 1;
          } else {
            armPositionSetting = 2;
          }
        }
      }

      
      //set last_pov for next loop
      last_pov = gamePad1.getPOV();

      SmartDashboard.putNumber("Arm Position Setting", armPositionSetting);

      //sets LEDs to Yellow
      /*if(gamePad1.getRawButton(4)) {
        m_LedSubsystem.setColor(true);
      }*/

      if(gamePad1POVUp.getAsBoolean()) {

      }

      joystick1Button2.whileTrue(m_armCommand);
      joystick1Button3.whileTrue(m_armNegCommand);
      joystick1Button11.whileTrue(m_armExtendCommand);
      joystick1Button10.whileTrue(m_armRetractCommand);
      joystick1Button6.whileTrue(m_TurntableLeftCommand);
      joystick1Button7.whileTrue(m_TurntableRightCommand);
      joystick2Button5.whileTrue(m_intakeCubeCommand);
      joystick2Button4.whileTrue(m_intakeConeCommand);
      joystick2Button2.whileTrue(m_intakeReverseCommand);
      joystick2Button3.whileTrue(m_intakePositionCommand);

      gamePad1Button1.whileTrue(m_gripperReleaseCommand);
      gamePad1Button3.whileTrue(m_gripperCubeCommand);
      gamePad1Button4.whileTrue(m_gripperConeCommand);


      gamePad1Button8.whileTrue(m_armSelectedPositionCommand);

      if(gamePad1.getRawButtonPressed(10)) {
        armManualMode = !armManualMode;
        SmartDashboard.putBoolean("Manual Mode", armManualMode);
      }

      //Makes sure the armManualCommand only runs when joystick is in use
      if(Math.abs(getRightY()) > .2) {
        if(!m_armManualCommand.isScheduled()) {
          m_armManualCommand.schedule();
        }
        
      } else if(Math.abs(getRightX()) > .3) {
        if(!m_armManualCommand.isScheduled()) {
          m_armManualCommand.schedule();
        }
      } else {
        if(m_armManualCommand.isScheduled()) {
          m_armManualCommand.cancel();
        }
      }
    }




/* 

    public void resetDriveEncoders() {
        m_robotDrive.resetEncoders();
    }

*/

  
  
  
  
  
  


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


}
