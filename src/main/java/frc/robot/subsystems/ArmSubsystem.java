package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {
    CANSparkMax arm;
    RelativeEncoder encoder;
    int subArmiD = 31; //change id to correct id later

    //65 rotations is full extention for extending arm
    
    public ArmSubsystem() {
        arm = new CANSparkMax(subArmiD, MotorType.kBrushless);
        encoder = arm.getEncoder();
    }

    public void rotate(double speed) {
        arm.set(speed);
        
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
    }

    /*public void arm(double speed) {
        armMotor.set(speed);
   } */
}
