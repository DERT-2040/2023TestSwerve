package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;;

public class GripperSubsystem extends SubsystemBase {
    double m_count = 0;
    double m_prevCount = 0;
    Counter counter;
    Spark gripperTalon;
    public GripperSubsystem() {
        counter = new Counter(4);
        gripperTalon = new Spark(0);
        counter.reset();
 // Set up the input channel for the counter
  //counter.setUpSource(4);

 // Set the encoder to count pulse duration from rising edge to falling edge
 
    }

    public void grip(double power) {

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
        SmartDashboard.putData("Gripper Turn 180", moveGrip(180, 1));
    }

    public Sendable moveGrip(int counts, double power) {
        double abs_count = Math.abs(m_count);
        while (abs_count > (m_count - counts) || abs_count < (m_count + counts)) {
            grip(power);
        }
        return counter;
    }
    
    public void gripObject() {
        
    }
}