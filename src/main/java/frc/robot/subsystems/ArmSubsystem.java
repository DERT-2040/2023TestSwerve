package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.NavigableMap;

import com.revrobotics.SparkMaxAbsoluteEncoder;

public class ArmSubsystem extends SubsystemBase {
    static double m_count = 0;
    double m_prevCount = 0;
    Counter counter;
    Spark gripperTalon;
    Spark armExtendNeo;
    Spark armRotateNeo;
    public void ArmSubsystem() {
        counter = new Counter(4);
        gripperTalon = new Spark(0);
        armExtendNeo = new Spark(0);
        armRotateNeo = new Spark(0);
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

    public void setExtension (int location) {
        if (location > armExtendNeo.get()) {
            armExtendNeo.set(1);
        } else if (location < armExtendNeo.get()) {
            armExtendNeo.set(-1);
        }
    }

    public void setRotation (int location) {
        if (location > armRotateNeo.get()) {
            armRotateNeo.set(1);
        } else if (location < armRotateNeo.get()) {
            armRotateNeo.set(-1);
        }
    }
    
}