// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
//import frc.robot.Constants.ModuleConstants;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {  // Class Definition  **************************************************************
  private CANSparkMax m_driveMotor;
    //Turret motor controller encoder
  private RelativeEncoder m_driveEncoder;
    
  private final CANSparkMax m_turningMotor;

  private final DutyCycleEncoder m_turningEncoder;

  private String turnOffsetKey;

  private double turnOffset;

//  private int turningReversed = 1;
  private SparkMaxPIDController m_drivePIDController;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.7,
          0.005,
          0.000001,
          new TrapezoidProfile.Constraints(
              20,
              20));

  /**
   * Constructs a SwerveModule.
   * @param turnOffsetKey Text Key to save and retrieve the turing encoder offset
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(     // Constructor 
      String turnOffsetKey,
      int driveMotorChannel,
      int turningMotorChannel,
      //int driveEncoderChannel,
      int turningEncoderChannel,
      Boolean driveEncoderReversed,
      Boolean turningEncoderReversed) {

    this.turnOffsetKey = turnOffsetKey;

    if(!Preferences.containsKey(this.turnOffsetKey)){                     // check to see if the wheel offset has been saved
      Preferences.setDouble(this.turnOffsetKey, 0);                // if it has not save a 0 offset
    }
    turnOffset = Preferences.getDouble(this.turnOffsetKey, 0);  // read the wheel angle offset stored in memory  

    

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannel);

    m_drivePIDController = m_driveMotor.getPIDController();
        kP = 0.00005;
        kI = 0;
        kD = 0.000001;
        kIz = 0;
        kFF = 0.00025;
        kMaxOutput = 1;
        kMinOutput = 0;

        // set PID coefficients
        m_drivePIDController.setP(kP);
        m_drivePIDController.setI(kI);
        m_drivePIDController.setD(kD);
        m_drivePIDController.setIZone(kIz);
        m_drivePIDController.setFF(kFF);
        //m_drivePIDController.setOutputRange(kMinOutput, kMaxOutput);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(.0398);
    //m_driveEncoder.setDistancePerRotation(1);
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.???
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);
        //m_driveEncoder.setInverted(driveEncoderReversed);
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setDistancePerRotation(Math.PI*2);   //EWO changed 12 to 3.14159*2

    // Set whether turning encoder should be reversed or not
   // if(turningEncoderReversed) {turningReversed = -1;}
   // m_turningEncoder.setReverseDirection(turningEncoderReversed);


   // Set the end stops for the encoder (min duty cycle and max duty cylce at the point the PWM "wraps" around)
   // m_turningEncoder.setDutyCycleRange(1/4096, 4095/4096); 

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

   /*  removed unused in rest of code EWO 1/26/23
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(GetTurningEncoderValue()));   // was getDistance
  }
  */
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(GetTurningEncoderValue()));   // was getDistance
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(GetTurningEncoderValue()));   // was getDistance

    // Calculate the drive output from the drive PID controller.
    // velocity meter/sec * (60 sec/min * 8.14 gear ratio) / (0.1016 meters * pi) = V * 1530 rev/min
    // speed request meters.sec * (60 sec/min * 8.14 motor rev / wheel rev) / (0.1016*pi meters/ wheel rev)
    m_drivePIDController.setReference(1530 * state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    //SmartDashboard.putNumber("Position", GetTurningEncoderValue());   // was getDistance
    //SmartDashboard.putNumber("Target", state.angle.getRadians());
    //SmartDashboard.putNumber("Velocity", m_turningEncoder.get);
    SmartDashboard.putNumber(turnOffsetKey + " Drive Output Current", m_driveMotor.getOutputCurrent());
    SmartDashboard.putNumber(turnOffsetKey + " Turn Output Current", m_turningMotor.getOutputCurrent());
    SmartDashboard.putNumber(turnOffsetKey + " Drive Temp F", m_driveMotor.getMotorTemperature() * (9/5) + 32);
    SmartDashboard.putNumber(turnOffsetKey + " Turn Temp F", m_turningMotor.getMotorTemperature() * (9/5) + 32);
    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(GetTurningEncoderValue(), state.angle.getRadians());//  was getDistance
        /* 
        SmartDashboard.putNumber("Radians", state.angle.getRadians());
        SmartDashboard.putNumber("output", turnOutput);

        SmartDashboard.putNumber(turnOffsetKey + "1", (GetTurningEncoderValue()));
        SmartDashboard.putNumber(turnOffsetKey + "Drive", m_driveEncoder.getVelocity());*/

     //Calculate the turning motor output from the turning PID controller.
    m_turningMotor.set(-turnOutput);
  }



 
  public String GetTurnOffsetKey() {
    return turnOffsetKey;
  }


  public double GetTurningCalibrationValue() {
    return (m_turningEncoder.getAbsolutePosition() * 2 * Math.PI) % (2 * Math.PI);

  }


  public double GetTurningEncoderValue() {
    return ((GetTurningCalibrationValue()) - turnOffset + 16 * Math.PI) % (2 * Math.PI);  //was getDistance
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.reset();
  }

}
