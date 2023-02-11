package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    CANSparkMax armMotor;
    int armID = 31; 
    public ArmSubsystem() {
        armMotor = new CANSparkMax(armID, MotorType.kBrushless);
    }

    public void arm(double speed) {
        armMotor.set(speed);
    } 
}
