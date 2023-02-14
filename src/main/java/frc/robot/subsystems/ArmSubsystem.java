package frc.robot.subsystems;
<<<<<<< HEAD

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

=======
import com.revrobotics.CANSparkMax;
>>>>>>> Feb-10
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {
<<<<<<< HEAD
    CANSparkMax armMotor;
    int armID = 31; 
    public ArmSubsystem() {
        armMotor = new CANSparkMax(armID, MotorType.kBrushless);
=======
    CANSparkMax armSubsystem;
    int subArmiD = 5; //change id to correct id later
    public ArmSubsystem() {
        armSubsystem = new CANSparkMax(subArmiD, MotorType.kBrushless);
    }

    public void armSubsystem(double speed) {
        armSubsystem.set(speed);
>>>>>>> Feb-10
    }

    public void arm(double speed) {
        armMotor.set(speed);
    } 
}
