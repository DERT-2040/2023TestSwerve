package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveControlSubsystem extends SubsystemBase {
    DriveSubsystem m_robotDrive;
    VisionSubsystem m_visionSubsystem;

    double filteredX;
    double filteredY;
    double speedBoost;
    AHRS ahrs;
    

    PIDController balancingPID;
    
    public DriveControlSubsystem(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_robotDrive = driveSubsystem;
        m_visionSubsystem = visionSubsystem;

        filteredX = 0;
        filteredY = 0;
        
        balancingPID = new PIDController(1, 0, 0);
        ahrs = RobotContainer.getAHRS();
    }

    public void balance() {
      
      if(ahrs.getRoll() < -8) {
        double diff = Math.abs((ahrs.getRoll() + 8) * 0.05);
        if(diff > 1) {diff = 1;}
        simpleDrive(0, .9 * diff, 0, true);
        //SmartDashboard.putNumber("Balancing PID", balancingPID.calculate(ahrs.getRoll(), 0));
      } else if(ahrs.getRoll() > 8) {
        double diff = Math.abs((ahrs.getRoll() - 8) * 0.05);
        if(diff > 1) {diff = 1;}
        simpleDrive(0, -.9 * diff, 0, true);
      } else {
        simpleDrive(0, 0, 0.0001, true);
      }
    }
      

    /** Positive X: Left, Positive Y: Back */
    public void simpleDrive(double x, double y, double rot, boolean fieldRelative) {
      //filtering could be removed if needed
      /*double filter = 0.25;
          filteredX = (x - filteredX) * filter + filteredX;
          x = filteredX;
          filteredY = (y - filteredY) * filter + filteredY;
          y = filteredY;
*/
      m_robotDrive.drive(x, y,  rot, fieldRelative);
    }


    /** angle of 0 (hopefully) is straight forward
     *  Positive X: Left, Positive Y: Back
     */
    public void simpleDriveToRotation(double x, double y, double angle, boolean fieldRelative) {
      double rot = (m_robotDrive.getPose().getRotation().getDegrees() - angle) / (180);
          
      if(rot > .5) {
        rot = .5;
      } else if(rot < -.5) {
        rot = -.5;
      }

      m_robotDrive.drive(x, y,  rot, fieldRelative);
      
    }


    



    public void drive(double x, double y, double rot, double gamePadX, double gamePadY, boolean boostTrigger, boolean superSpeed, boolean lock, boolean auto, boolean balancing) {
      
        //double executionTime = Timer.getFPGATimestamp();
        
  
      //Joystick values
        /*double x = -joystick1.getX();
        double y = joystick1.getY();
        double rot = joystick2.getX();*/
        double deadband = 0.15;
        if(x > -deadband && x < deadband) {
          x = 0;
        } else {
            if(x > 0) {
                x = (x - deadband) / (1 - deadband);
            } else {
                x = (x + deadband) / (1 - deadband);
            }
            
        }

        if(y > -deadband && y < deadband) {
          y = 0;
        } else {
            if(y > 0) {
                y = (y - deadband) / (1 - deadband);
            } else {
                y = (y + deadband) / (1 - deadband);
            }
        }

        if(rot > -deadband && rot < deadband) {
          rot = 0;
        } else {
            if(rot > 0) {
                rot = (rot - deadband) / (1 - deadband);
            } else {
                rot = (rot + deadband) / (1 - deadband);
            }
        }


        if(x > 0) {
            x = Math.pow(x, 2);
        } else if(x < 0) {
            x = -Math.pow(x, 2);
        }

        if(y > 0) {
            y = Math.pow(y, 2);
        } else if(x < 0) {
            y = -Math.pow(y, 2);
        }

        if(rot > 0) {
            rot = Math.pow(rot, 2);
        } else if(x < 0) {
            rot = -Math.pow(rot, 2);
        }
        
  
        /*  Next step should be to convert getVision / getPose from returning translation from camera to the April Tag to
         *  the actual field position
         *  then here we can define a desired filed location
         *  and calculate the translation from current position to the desired and auto travel that path
         * 
         *  Future step will be to use the photon vision library to merge the april tag location with the swerve obometry position
         */
  
        if(/*joystick2Button8.getAsBoolean()*/auto) {
  
        
          double m_maximum = 1;
  
          Pose2d pose = getVision();
          x = pose.getX();
          y = pose.getY();
          
  
          double test = Math.max(Math.abs(x),Math.abs(y));
          if(test > m_maximum){
            x = x * m_maximum / test;
            y = y * m_maximum / test;
          }
  
          
          
          
          rot = pose.getRotation().getDegrees() / (180);
          
          if(rot > .5) {
            rot = .5;
          } else if(rot < -.5) {
            rot = -.5;
          }
        }
  
  
        //Vertical axis
        //SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        //sideways axis
        //SmartDashboard.putNumber("Pitch", ahrs.getPitch());
        //forward camera facing axis
        //SmartDashboard.putNumber("Roll", ahrs.getRoll());
        double speed = 1.5;//(-joystick1.getZ() + 1) / 2;
        
        /*if(RobotController.getBatteryVoltage() < 10) {
          speed = 0;
        }*/

        double filter = 0.25;
          filteredX = (x - filteredX) * filter + filteredX;
          x = filteredX;
          filteredY = (y - filteredY) * filter + filteredY;
          y = filteredY;
  
        if(Math.abs(/*gamePad1.getRawAxis(0)*/ gamePadX) > .1) {
          x += .25 * -gamePadX;
          if(x > 1) {
            x = 1;
          } else if(x < -1) {
            x = -1;
          }
        }
        if(Math.abs(/*gamePad1.getRawAxis(1)*/ gamePadY) > .1) {
          y += .25 * gamePadY;
          if(y > 1) {
            y = 1;
          } else if(y < -1) {
            y = -1;
          }
        }
        
        double rotSpeed = 2;
        if(superSpeed) {
            speed = 4;
            rotSpeed = 3;
        } else if(boostTrigger) {
          /*if(speedBoost < 1.5) {
            speedBoost += .05;
          } else {
            speedBoost = 1.5;
          }
            speed += speedBoost;
            if(speed > 3) {
              speed = 3;
            }*/
            speed = 3;
            rotSpeed = 3;
        }/* else {
          if(speedBoost > 0) {
            speedBoost -= .05;
          } else {
            speedBoost = 0;
          }
          speed += speedBoost;
            if(speed > 3) {
              speed = 3;
            }

        }*/

        if(lock) {
          speed = 0;
          rotSpeed = 1;
          rot = .0001;
        }

        
  
        m_robotDrive.drive(speed * x, speed * y, rotSpeed * rot, true);
        //SmartDashboard.putNumber("Drive Execution Time", Timer.getFPGATimestamp() - executionTime);
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
  
          //SmartDashboard.putNumber("x error",m_xError);
          //SmartDashboard.putNumber("y error",m_yError);
  
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
}
