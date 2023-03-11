package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;


public class ArmSubsystem extends SubsystemBase {

    CANSparkMax arm;
    DutyCycleEncoder rotateEncoder;
    PIDController rotateControl;

    Spark gripperTalon;
    //DigitalInput gripperLimitSwitch;
    DutyCycleEncoder gripperEncoder;
    PIDController gripperController;
 
 
    CANSparkMax armExtendNeo;
    RelativeEncoder extendEncoder;
    PIDController extendControl;

    //current arm target position setting
    int armSetting;

    double[] rotateAngles =        {-200,   -50,   -20,   0,      30,    45,     70,     90,     200};
    double[] extendDistancesHigh = {0.014,  0.014, 0.014, 0.014,  0.014, 0.014,  0.014,  .9,     .9};
    double[] extendDistancesMid =  {0.014,  0.014, 0.014, 0.014,  0.014, 0.014,  0.014,  0.014,  0.014};
    double[] extendDistancesLow =  {0.014,  0.014, 0.014, 0.014,  .6,    .9,     .9,     .9,     .9};
    double[] extendGrab =          {0.5,  0.5, 0.5, 0.014,  0.014, 0.014,  0.014,  0.014,  0.014};

    public ArmSubsystem() {
        //65 rotations is full extention for extending arm
        //55 rotations = 90 degrees arm rotation

        //m_count = 0;
        //m_prevCount = 0;

        //counter = new Counter(4);
        gripperTalon = new Spark(0);
        //gripperLimitSwitch = new DigitalInput(5);
        gripperEncoder = new DutyCycleEncoder(5);
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
        extendEncoder.setPosition(.41 * 58);
        extendControl = new PIDController(1.5, 0, 0.1);


        

   //     rotateControl = new PIDController(.003, .001, 0);


        //counter.reset();
 
    }


    public void grip_speed(double power) {
        double gripPosition = (gripperEncoder.getAbsolutePosition() - .3) * 3.33333;
        SmartDashboard.putNumber("Gripper Encoder Position", gripPosition);
        SmartDashboard.putNumber("Grip Raw Pos", gripperEncoder.getAbsolutePosition());


        
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
        } else if(gripPosition > 0.98) {
            if(power > 0) {
                power = 0;
            }
        }

        //SmartDashboard.putBoolean("GripperLimit", !gripperLimitSwitch.get());
        // m_prevCount = m_currentCount;

        gripperTalon.set(power);
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
        if((((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27) > 100 && speed < 0) || ((((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27) < -50) && speed > 0)) {
            arm.set(0);
        } else {
            
            
            arm.set(speed);
            SmartDashboard.putNumber("Known Arm Position", (rotateEncoder.getAbsolutePosition() - .17) * 90 / .27);
        }
    }  



    //rotates arm by speed and extends automatically
    public void rotate(double speed) { // from Feb24 code
        //PRevents the arm from going higher than a certain amount
        if((((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27) > 100 && -speed < 0) ||
           (((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27) < -50) && -speed > 0) {
            arm.set(0);
        } else {
           
           
            arm.set(-speed);
            extendAutomatically(((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27));
            //SmartDashboard.putNumber("Arm Position", rotateEncoder.getPosition());
        }
        SmartDashboard.putNumber("Known Arm Position", (rotateEncoder.getAbsolutePosition() - .17) * 90 / .27);

  //      SmartDashboard.putNumber("Actual Arm Angle", (rotateEncoder.getAbsolutePosition() - .76 ) * 352);
    }

    private double getArmRotation(){
        return((rotateEncoder.getAbsolutePosition() - .17) * 90 / .27);
    }


    //sets the arm angle with default setting of high   
    public void setArmAngle(double angle) {
        //armSetting = 2;
        double actualArmAngle = (rotateEncoder.getAbsolutePosition() - .17) * 90 / .27; //* (90/60) * (90.0/50.0) * 2;
        rotate(rotateControl.calculate(actualArmAngle, angle));
    //    SmartDashboard.putNumber("Actual Arm Angle", (rotateEncoder.getAbsolutePosition() - .76 ) * 352);
        
    //    SmartDashboard.putNumber("Desired Arm Angle", angle);
        
    }


    //sets the arm angle and extend based on provided setting
    public void setArmWithSetting(int setting) {
        armSetting = setting;
        double actualArmAngle = getArmRotation(); //* (90/60) * (90.0/50.0) * 2;
        if(setting == 0) {
            rotate(rotateControl.calculate(actualArmAngle, 30));
            SmartDashboard.putNumber("Desired Arm Angle", 30);
        } else if(setting == 1) {
            rotate(rotateControl.calculate(actualArmAngle, 70));
            SmartDashboard.putNumber("Desired Arm Angle", 70);
        } else if(setting == 2) {
            rotate(rotateControl.calculate(actualArmAngle, 90));
            SmartDashboard.putNumber("Desired Arm Angle", 90);
        } else {
            rotate(rotateControl.calculate(actualArmAngle, -20));
            SmartDashboard.putNumber("Desired Arm Angle", -20);
        }
        

    }


    public void setExtendPosition(double position) {
        setExtendSpeed(extendControl.calculate(extendEncoder.getPosition() / 57, position));
        SmartDashboard.putNumber("Desired Arm Extend", position);
        
    }


    public void setExtendSpeed(double speed) {
        if(extendEncoder.getPosition() / 57 > .9) {
            if(speed > 0) {
                speed = 0;
            }
        } else if(extendEncoder.getPosition() / 57 < 0) {
            if(speed < 0) {
                speed = 0;
            }
        }
        armExtendNeo.set(speed);
        SmartDashboard.putNumber("Actual Arm Extend", (extendEncoder.getPosition() / 57));
        
    }


    //allows you to choose setting
    public void extendAutomatically(double actualArmAngle) {
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
        SmartDashboard.putNumber("extend Position", extend);
        //armExtendNeo.set(extendControl.calculate(extendEncoder.getPosition() / 58, extend));
        //extendControl.setReference(extend, ControlType.kPosition);*/
    }
    
}
