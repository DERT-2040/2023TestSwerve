package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.NavigableMap;

import org.apache.commons.lang3.function.TriFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {
    static double m_count = 0;
    double m_prevCount = 0;
    Counter counter;
    Spark gripperTalon;
    CANSparkMax armExtendNeo;
    RelativeEncoder extendEncoder;
    SparkMaxPIDController extendControl;
    CANSparkMax armRotateNeo;
    RelativeEncoder rotateEncoder;
    SparkMaxPIDController rotateControl;




    double[] rotateAngles = {};
    double[] extendDistances = {};



    public void ArmSubsystem() {
        counter = new Counter(4);
        gripperTalon = new Spark(0);
        armExtendNeo = new CANSparkMax(30, MotorType.kBrushless);
        extendEncoder = armExtendNeo.getEncoder();
        extendControl = armExtendNeo.getPIDController();
        extendControl.setP(1);
        extendControl.setI(0);
        extendControl.setD(0);
        extendControl.setIZone(0);
        extendControl.setFF(0);
        armRotateNeo = new CANSparkMax(31, MotorType.kBrushless);
        rotateEncoder = armRotateNeo.getEncoder();
        rotateControl = armRotateNeo.getPIDController();
        rotateControl.setP(1);
        rotateControl.setI(0);
        rotateControl.setD(0);
        rotateControl.setIZone(0);
        rotateControl.setFF(0);
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
        }
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

    public boolean setExtend (int location) {
        if (location > armExtendNeo.get()) {
            armExtendNeo.set(1);
        } else if (location < armExtendNeo.get()) {
            armExtendNeo.set(-1);
        } else if (location == armExtendNeo.get()) {
            return false;
        }
        return true;
    }

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

    public void runPresetTable (int[] extendList, int[] rotateList) {
        for (int i = 0;i < extendList.length; i++) {
            boolean rotateDone = false;
            boolean extendDone = false;
            if (rotateDone && extendDone != true) {
            if (rotateList[i] != armRotateNeo.get()) {
                setRotation(rotateList[i]);
            } else {
                rotateDone = true;
            }
            if (extendList[i] != armExtendNeo.get()) {
                setExtend(extendList[i]);
            } else {
                rotateDone = true;
            }
        }
        }

        boolean frameSatisfied = false;
        //if (frameSatisfied != true)
    }
    public void setArmAngle(double angle) {
        rotateControl.setReference(angle, ControlType.kPosition);
        int i = 0;
        while(angle > rotateAngles[i]) {
            i++;
        }

        double extend = extendDistances[i-1] + ((angle - rotateAngles[i-1]) / (rotateAngles[i] - rotateAngles[i-1])) * (extendDistances[i] - extendDistances[i-1]);
        extendControl.setReference(extend, ControlType.kPosition);
    }

    
}