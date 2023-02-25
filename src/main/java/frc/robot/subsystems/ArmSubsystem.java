package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
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
    RelativeEncoder rotateEncoder;
    PIDController rotateControl;
    int subArmiD = 40; //change id to correct id later

 
  //  static double m_count = 0;
  //  double m_prevCount = 0;
  //  Counter counter;
  //  Spark gripperTalon;
  //  DigitalInput gripperLimitSwitch;
 
 
    CANSparkMax armExtendNeo;
    RelativeEncoder extendEncoder;
    PIDController extendControl;

    double[] rotateAngles =    {-90, -70,  0,    50};
    double[] extendDistances = {-.9, -.3, -0.1, -0.1};

    public ArmSubsystem() {
        //65 rotations is full extention for extending arm
        //55 rotations = 90 degrees arm rotation

    //    counter = new Counter(4);
    //    gripperTalon = new Spark(0);
    //    gripperLimitSwitch = new DigitalInput(5);

        
        arm = new CANSparkMax(subArmiD, MotorType.kBrushless);
        arm.restoreFactoryDefaults();
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSecondaryCurrentLimit(40);
        arm.setSmartCurrentLimit(40);
        arm.setSmartCurrentLimit(40,5700);
        arm.setOpenLoopRampRate(0.75);
        rotateEncoder = arm.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rotateEncoder.setPositionConversionFactor(90.0/107.0);
        rotateEncoder.setPosition(0);
        rotateControl = new PIDController(.003, .001, 0);
        



        armExtendNeo = new CANSparkMax(30, MotorType.kBrushless);
        armExtendNeo.restoreFactoryDefaults();
        armExtendNeo.setIdleMode(IdleMode.kBrake);
        armExtendNeo.setSecondaryCurrentLimit(20);
        armExtendNeo.setSmartCurrentLimit(20);
        armExtendNeo.setSmartCurrentLimit(20,10000);
        armExtendNeo.setOpenLoopRampRate(0.75);
        extendEncoder = armExtendNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        extendEncoder.setPosition(-.4 * 58);
        extendControl = new PIDController(1.5, 0, 0.1);


        




        



   //     rotateControl = new PIDController(.003, .001, 0);





    //    counter.reset();
 
    }

/* 
    public void grip_speed(double power) {

        
        double m_currentCount = counter.get();
    ;
        if (power > 0) {
            m_count = m_count + (m_currentCount - m_prevCount);
         } else {
            m_count = m_count - (m_currentCount - m_prevCount);
         }
         if(!gripperLimitSwitch.get()) {
            m_count = 0;
            if(power < 0) {
                power = 0;
            }
        }
        SmartDashboard.putBoolean("GripperLimit", !gripperLimitSwitch.get());
         m_prevCount = m_currentCount;

        gripperTalon.set(power);
        SmartDashboard.putNumber("Counter", m_count);
        SmartDashboard.putNumber("Gripper Power",power);
    }

    public void grip_goto(int location) {
        if (location > m_count) {
            grip_speed(1);
        } else if (location < m_count) {
            grip_speed(-1);
        } else {
            grip_speed(0);
        }
        SmartDashboard.putNumber("Counter", m_count);
    }

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

    public void rotate(double speed) { // from Feb24 code
        arm.set(speed);
        
        //SmartDashboard.putNumber("Arm Position", rotateEncoder.getPosition());
        SmartDashboard.putNumber("Actual Arm Angle", rotateEncoder.getPosition());
    }

 
    public void setExtendPosition(double position) {
        armExtendNeo.set(extendControl.calculate(extendEncoder.getPosition() / 57, position));
        SmartDashboard.putNumber("Desired Arm Extend", position);
    }


    public void setExtendSpeed(double speed) {
        armExtendNeo.set(speed);
        SmartDashboard.putNumber("Desired Arm Extend", (extendEncoder.getPosition() / 57));
        
    }

    public void setArmAngle(double angle) {
        double actualArmAngle = rotateEncoder.getPosition(); //* (90/60) * (90.0/50.0) * 2;
        arm.set(rotateControl.calculate(actualArmAngle, angle));
        SmartDashboard.putNumber("Actual Arm Angle", actualArmAngle);
        /*if(actualArmAngle < -90) {
            actualArmAngle = -90;
        } else if(actualArmAngle > 50) {
            actualArmAngle = 50;
        }
        //rotateControl.setReference(angle, ControlType.kPosition);
        int i = 0;
        while(actualArmAngle > rotateAngles[i] && i < rotateAngles.length) {
            i++;
        }

        double extend = extendDistances[i-1] + ((actualArmAngle - rotateAngles[i-1]) / (rotateAngles[i] - rotateAngles[i-1])) * (extendDistances[i] - extendDistances[i-1]);
        setExtendPosition(extend);
        //armExtendNeo.set(extendControl.calculate(extendEncoder.getPosition() / 58, extend));
        //extendControl.setReference(extend, ControlType.kPosition);*/
    }
    
}
