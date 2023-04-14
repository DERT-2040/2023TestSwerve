package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Counter;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;


public class ArmSubsystem extends SubsystemBase {

    CANSparkMax arm;
    DutyCycleEncoder rotateEncoder;
    PIDController rotateControl;
    public Boolean armInPosition;

    Spark gripperTalon;
    //DigitalInput gripperLimitSwitch;
    DutyCycleEncoder gripperEncoder;
    PIDController gripperController;
    public Boolean gripInPosition;
 
 
    CANSparkMax armExtendNeo;
    RelativeEncoder extendEncoder;
    PIDController extendControl;
    public Boolean extendInPosition;

    //current arm target position setting
    int armSetting;

    double[] rotateAngles =        {-200,   -50,   -20,  -10,   0,    15,  30,    45,     70,     90,     200};
    double[] extendDistancesHigh = {0.014,  0.014, 0.014,  0.014, 0.014, 0.014,   0.014, 0.014,  0.014,  .9,     .9};
    double[] extendDistancesMid =  {0.014,  0.014, 0.014,  0.014, 0.014,  0.014,   0.014, 0.014,  0.014,  0.014,  0.014};
    double[] extendDistancesLow =  {0.51,   0.51, 0.42,   0.29,  0.23, .33,  .17,   .33,    .9,     .9,     .9,     .9};
    double[] extendGrab =          {0.51,   0.51, 0.51,   0.51,  0.014, 0.014,   0.014, 0.014,  0.014,  0.014,  0.014};
    double[] extendLimit =         {0.53,   0.56, 0.46,   0.31,  0.25, .35,  .17,   .33,    1,    1,    1};
    

    public ArmSubsystem() {
        //65 rotations is full extention for extending arm
        //55 rotations = 90 degrees arm rotation

        //m_count = 0;
        //m_prevCount = 0;

        //counter = new Counter(4);
        gripperTalon = new Spark(0);
        //gripperLimitSwitch = new DigitalInput(5);
        gripperEncoder = new DutyCycleEncoder(4);
        gripperController = new PIDController(12, .001, 0);

        gripperTalon.set(0);

        
        arm = new CANSparkMax(45, MotorType.kBrushless);
        arm.restoreFactoryDefaults();
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSecondaryCurrentLimit(40);
        arm.setSmartCurrentLimit(40);
        //arm.setSmartCurrentLimit(40,5700);
        arm.setOpenLoopRampRate(0.75);
        rotateEncoder = new DutyCycleEncoder(6);
        /*rotateEncoder = arm.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rotateEncoder.setPositionConversionFactor(90.0/107.0);
        rotateEncoder.setPosition(0);*/
        rotateControl = new PIDController(.015, .01, 0);
        rotateControl.setIntegratorRange(-.1, .1);

        armSetting = 2;
        



        armExtendNeo = new CANSparkMax(30, MotorType.kBrushless);
        armExtendNeo.restoreFactoryDefaults();
        armExtendNeo.setIdleMode(IdleMode.kBrake);
        armExtendNeo.setSecondaryCurrentLimit(20);
        armExtendNeo.setSmartCurrentLimit(20);
        //armExtendNeo.setSmartCurrentLimit(20,10000);
        armExtendNeo.setOpenLoopRampRate(0.75);
        extendEncoder = armExtendNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        extendEncoder.setPosition(/*.29*/ .34 * 58);
        extendControl = new PIDController(2, 0, 0.2);


        

   //     rotateControl = new PIDController(.003, .001, 0);


        //counter.reset();
 
    }


    public void grip_speed(double power) {
        double gripPosition = (gripperEncoder.getAbsolutePosition() - .2) * 3.33333;
        //SmartDashboard.putNumber("Gripper Encoder Position", gripPosition);
        //SmartDashboard.putNumber("Grip Raw Pos", gripperEncoder.getAbsolutePosition());


        
        //double m_currentCount = counter.get();
    
        /*if (power > 0) {
            m_count = m_count + (m_currentCount - m_prevCount);
         } else {
            m_count = m_count - (m_currentCount - m_prevCount);
         }*/
        if(gripPosition < 0.15) {
            if(power < 0) {
                power = 0;
            }
        } else if(gripPosition > 1.25) {
            if(power > 0) {
                power = 0;
            }
        }

        //SmartDashboard.putBoolean("GripperLimit", !gripperLimitSwitch.get());
        // m_prevCount = m_currentCount;

        gripperTalon.set(power);
        SmartDashboard.putNumber("Grip", (gripperEncoder.getAbsolutePosition() - .3) * 3.33333);
        //SmartDashboard.putNumber("Counter", m_count);
        //SmartDashboard.putNumber("Current Count", counter.get());
    }

    //0 to 1
    public void grip_goto(double location) {
        /*if (location > m_count) {
            grip_speed(.1);
        } else if (location < m_count) {
            grip_speed(-.1);
        } else {
            grip_speed(0);
        }*/
        //SmartDashboard.putNumber("Counter", m_count);

        grip_speed(gripperController.calculate((gripperEncoder.getAbsolutePosition() - .3) * 3.33333, location));
        
        if(Math.abs((gripperEncoder.getAbsolutePosition() - .3) * 3.33333 - location) < .1) {
            gripInPosition = true;
        } else {
            gripInPosition = false;
        }
        
    }

    /*
    public void gripCone(boolean ButtonInput) {
        if (ButtonInput) {
        grip_goto(600);
        }
    }

    public void gripCube(boolean ButtonInput) {
        if (ButtonInput) {
        grip_goto(520);
        }
    }

    public void gripRelease(boolean ButtonInput) {
        if (ButtonInput) {
        grip_goto(2000);
        }
    }  */

    public void setSetting(int setting) {
        armSetting = setting;
    }

    //for arm rotation with joystick and no automatic extention
    public void manualRotateArm(double speed) {
        if(((getArmRotation()) > 95 && speed < 0) ||
        ((getArmRotation()) < -25) && speed > 0) {
            arm.set(0);
        } else {
            
            
            arm.set(speed);
            
        }
        setExtendToLimit(.01);
        SmartDashboard.putNumber("Arm Position", getArmRotation());
    }  



    //rotates arm by speed and extends automatically
    public void rotate(double speed) { // from Feb24 code
        //PRevents the arm from going higher than a certain amount
        if(((getArmRotation()) > 95 && -speed < 0) ||
           ((getArmRotation()) < -50) && -speed > 0) {
            arm.set(0);
        } else {
           
           
            arm.set(-speed);
            extendAutomatically();
            
        }
        

  //      SmartDashboard.putNumber("Actual Arm Angle", (rotateEncoder.getAbsolutePosition() - .76 ) * 352);
    }

    private double getArmRotation(){
        double angle = (rotateEncoder.getAbsolutePosition() - .17) * 90 / .27;
        //SmartDashboard.putNumber("Known Arm Position", angle);
        return(angle);
    }


    //sets the arm angle with default setting of high   
    public void setArmAngle(double angle) {
        //armSetting = 2;
        double actualArmAngle = getArmRotation(); //* (90/60) * (90.0/50.0) * 2;
        rotate(rotateControl.calculate(actualArmAngle, angle));
        if(Math.abs(actualArmAngle - angle) < 2) {
            armInPosition = true;
        } else {
            armInPosition = false;
        }
        SmartDashboard.putNumber("Actual Arm Angle", (rotateEncoder.getAbsolutePosition() - .76 ) * 352);
        
    //    SmartDashboard.putNumber("Desired Arm Angle", angle);
        
    }


    //sets the arm angle and extend based on provided setting
    public void setArmWithSetting(int setting) {
        armSetting = setting;

        double setPoint;
        if(setting == 0) {
            setPoint = 30;
        } else if(setting == 1) {
            setPoint = 75;
        } else if(setting == 2) {
            setPoint = 95;
        } else {
            setPoint = -13;
        }

        setArmAngle(setPoint);

        //rotate(rotateControl.calculate(actualArmAngle, setPoint));
        //SmartDashboard.putNumber("Desired Arm Angle", setPoint);


    }

    public double getExtendPosition() {
        return extendEncoder.getPosition() / 57;
    }


    public void setExtendPosition(double position) {
        setExtendSpeed(extendControl.calculate(getExtendPosition(), position));
        //SmartDashboard.putNumber("Desired Arm Extend", position);

        if(Math.abs(getExtendPosition() - position) < .05) {
            extendInPosition = true;
        } else {
            extendInPosition = false;
        }
        
        
    }


    public void setExtendSpeed(double speed) {
        double position = getExtendPosition();
        if(position > .9) {
            if(speed > 0) {
                speed = 0;
            }
        } else if(position < 0) {
            if(speed < 0) {
                speed = 0;
            }
        }

        

        if(!setExtendToLimit(speed)) {
            armExtendNeo.set(speed);
        }
        
            
        

        //SmartDashboard.putNumber("Actual Arm Extend", (getExtendPosition()));
        
        
    }

    //returns true if the arm is past the limit
    private boolean setExtendToLimit(double speed) {
        int i = 0;
        while(getArmRotation() > rotateAngles[i] && i < rotateAngles.length) {
            i++;
        }

        double limit = extendLimit[i-1] + ((getArmRotation() - rotateAngles[i-1]) / (rotateAngles[i] - rotateAngles[i-1])) * (extendLimit[i] - extendLimit[i-1]);
        if(getExtendPosition() > limit && speed > 0) {
            armExtendNeo.set(extendControl.calculate(getExtendPosition(), limit));
            //SmartDashboard.putNumber("Limit Arm Extend", limit);
            return true;
        }
        return false;
        
    }


    //allows you to choose setting
    public void extendAutomatically() {
        double actualArmAngle = getArmRotation();
        //if(!setExtendToLimit()) {
            if(actualArmAngle > 90) {
                actualArmAngle = 90;
            } else if(actualArmAngle < -50) {
                actualArmAngle = -50;
            }
            //rotateControl.setReference(angle, ControlType.kPosition);
            int i = 0;
            while(actualArmAngle > rotateAngles[i] && i < rotateAngles.length) {
                i++;
            }

            double[] extendDistances;
            if(armSetting == 0) {
                extendDistances = extendDistancesLow;
            } else if(armSetting == 1) {
                extendDistances = extendDistancesMid;
            } else if(armSetting == 2) {
                extendDistances = extendDistancesHigh;
            } else {
                extendDistances = extendGrab;
            }

            double extend = extendDistances[i-1] + ((actualArmAngle - rotateAngles[i-1]) / (rotateAngles[i] - rotateAngles[i-1])) * (extendDistances[i] - extendDistances[i-1]);
            setExtendPosition(extend);
        //}
        //armExtendNeo.set(extendControl.calculate(extendEncoder.getPosition() / 58, extend));
        //extendControl.setReference(extend, ControlType.kPosition);*/
    }

    /*public void periodic() {
        SmartDashboard.putNumber("Arm Position", getArmRotation());
    }*/
    
}
