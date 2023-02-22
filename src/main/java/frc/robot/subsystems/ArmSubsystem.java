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


public class ArmSubsystem extends SubsystemBase {

    static double m_count = 0;
    double m_prevCount = 0;
    Counter counter;
    Spark gripperTalon;
    DigitalInput gripperLimitSwitch;


    CANSparkMax armExtendNeo;
    public RelativeEncoder extendEncoder;
    PIDController extendControl;
    public CANSparkMax armRotateNeo;
    public RelativeEncoder rotateEncoder;
    PIDController rotateControl;




    double[] rotateAngles =    {-90, -70,  0,    50};
    double[] extendDistances = {-.9, -.3, -0.1, -0.1};



    public ArmSubsystem() {
        //65 rotations is full extention for extending arm
        //55 rotations = 90 degrees arm rotation

        counter = new Counter(4);
        gripperTalon = new Spark(0);
        gripperLimitSwitch = new DigitalInput(5);

        
        armExtendNeo = new CANSparkMax(30, MotorType.kBrushless);
        armExtendNeo.setSmartCurrentLimit(5);
        armExtendNeo.setIdleMode(IdleMode.kBrake);
        extendEncoder = armExtendNeo.getEncoder();
        //extendEncoder.setPositionConversionFactor(1/(13000*58));
        
        extendEncoder.setPosition(-.4 * 58);
        extendControl = new PIDController(1.5, 0, 0.1);

        /*extendControl = armExtendNeo.getPIDController();
        extendControl.setP(.0001);
        extendControl.setI(0.00001);
        extendControl.setD(0);
        extendControl.setIZone(0);
        extendControl.setFF(0);
        extendControl.setOutputRange(-.1, .1);*/
        //extendControl.set
        




        armRotateNeo = new CANSparkMax(40, MotorType.kBrushless);
        armRotateNeo.setSmartCurrentLimit(45);
        armRotateNeo.setIdleMode(IdleMode.kBrake);
        
        rotateEncoder = armRotateNeo.getEncoder();
        //rotateEncoder.setPositionConversionFactor(90/60);
        rotateEncoder.setPosition(0);

        rotateControl = new PIDController(.003, .001, 0);



        /*
        rotateControl = armRotateNeo.getPIDController();
        rotateControl.setFeedbackDevice(rotateEncoder);

        rotateControl.setP(.000001);
        rotateControl.setI(0.00000);
        rotateControl.setD(0);
        //rotateControl.setIZone(0);
        rotateControl.setFF(0);
        rotateControl.setOutputRange(.3, .3);*/


        counter.reset();
    
 // Set up the input channel for the counter
  //counter.setUpSource(4);bbb

 // Set the encoder to count pulse duration from rising edge to falling edge
    }


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
    }

   /*  public boolean setExtend (int location) {
        if (location > armExtendNeo.get()) {
            armExtendNeo.set(1);
        } else if (location < armExtendNeo.get()) {
            armExtendNeo.set(-1);
        } else if (location == armExtendNeo.get()) {
            return false;
        }
        return true;
    }*/

    public boolean setRotation (int location) {
        if (location > armRotateNeo.get()) {
            armRotateNeo.set(1);
        } else if (location < armRotateNeo.get()) {
            armRotateNeo.set(-1);
        } else if (location == armRotateNeo.get()) {
            return false;
        }
        return true;
    }

    public void setArmAngle(double angle) {
        double actualArmAngle = rotateEncoder.getPosition() * (90/60) * (90.0/50.0);
        armRotateNeo.set(rotateControl.calculate(actualArmAngle, angle));
        SmartDashboard.putNumber("Target Arm Angle", angle);
        if(actualArmAngle < -90) {
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
        //extendControl.setReference(extend, ControlType.kPosition);
    }

    public void setArmSpeed(double speed) {
        armRotateNeo.set(speed);
    }

    //position from 0 (retracted) to 1 (extended)
    public void setExtendPosition(double position) {
        armExtendNeo.set(extendControl.calculate(extendEncoder.getPosition() / 58, position));
        SmartDashboard.putNumber("Desired Arm Extend", position);
    }


    public void setExtendSpeed(double speed) {
        armExtendNeo.set(speed);
    }
    
}