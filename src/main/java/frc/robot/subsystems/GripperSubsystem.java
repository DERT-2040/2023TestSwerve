package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;;

public class GripperSubsystem extends SubsystemBase {
    double counterPower;
    Counter counter;
    Spark gripperTalon;
    public GripperSubsystem() {
        counter = new Counter(4);
        gripperTalon = new Spark(0);
        counter.reset();
 // Set up the input channel for the counter
  //counter.setUpSource(4);

 // Set the encoder to count pulse duration from rising edge to falling edge
 if (counterPower > 0) {
    counter.setReverseDirection(false);
 } else {
    counter.setReverseDirection(true);
 }
 
    }

    public void grip(double power) {
        gripperTalon.set(power);
        SmartDashboard.putNumber("Counter", counter.get());
        counterPower = power;

    }
    public void moveGrip(int counts, char direction, double power) {
        int tempCounts = counter.get();
        int countTime = Math.abs(counter.get() - tempCounts);
        while(countTime>counts){
            if(direction == 'r') {
                gripperTalon.set(power);
            } else if(direction == 'l') {
                gripperTalon.set(-1 * power);
            } else {
                System.out.println("Invalid Gripper Command!");
            }
        }
    }
}
