// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SuperVisoryDrive extends SubsystemBase {
  /** Creates a new SuperVisoryDrive. */
  
  public SuperVisoryDrive() {}


  double filteredX = 0;
  double filteredY = 0;
/**
* @param stick_x Command (-1 to 1) positive is AWAY from the drive station
* @param stick_y Command (-1 to 1) positive is LEFT from the drive station
* @param rotate Command (-1 to 1) positive is counter clockwise
* @param auto  used to turn off joystick input and use auto drive to selected loacation
*/
  public void drv(double stick_x, double stick_y, double rotate, boolean autoDrive) {
  double x = stick_x;
  double y = stick_y;
  double rot = rotate;
  boolean auto = autoDrive;
  
  double executionTime = Timer.getFPGATimestamp();
  

//Joystick values
//  double x = -joystick1.getX();
//  double y = joystick1.getY();
//  double rot = joystick2.getX();
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

  //if(joystick2Button8.getAsBoolean()) {
  if(auto){
  
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

 RobotContainer.m_robotDrive.drive(speed * x, speed * y, speed * rot, true);
  SmartDashboard.putNumber("Drive Execution Time", Timer.getFPGATimestamp() - executionTime);



}


    // Target Pose is the desired location on the field to drive to
    Pose2d targetPose = new Pose2d(14, 2.75, new Rotation2d(0));
      
    PIDController m_xControl = new PIDController(2,0.2,0);
    PIDController m_yControl = new PIDController(1,0.2,0);



  //gets target position  preform drive to the targeted postion on the field when a button is pushed

  public Pose2d getVision() {

    m_xControl.setIntegratorRange(-0.1,0.1);
    m_yControl.setIntegratorRange(-0.1,0.1);

    Pose2d odometryPose = new Pose2d(RobotContainer.m_robotDrive.getPose().getX(), 
                                    RobotContainer.m_robotDrive.getPose().getY(), 
                                    RobotContainer.m_robotDrive.getPose().getRotation());
                                    
    Pose2d visionPose = RobotContainer.m_visionSubsystem.getPose();

    Pose2d fieldPose;


    //  if we have good data from vision use and average of vision and odometry otherwise just use odometery
    if(visionPose.getX() != -999){ 
      fieldPose = new Pose2d((odometryPose.getX() + visionPose.getX()) / 2, (odometryPose.getY() + visionPose.getY()) / 2, odometryPose.getRotation());
    } else{           
      fieldPose = odometryPose;
    }
    
    Pose2d robotToTarget = new Pose2d(  targetPose.getX() - fieldPose.getX(), 
                                        targetPose.getY() - fieldPose.getY(), 
                                        new Rotation2d(-targetPose.getRotation().getRadians() + fieldPose.getRotation().getRadians()));    

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
    RobotContainer.m_robotDrive.resetEncoders();

  }
}