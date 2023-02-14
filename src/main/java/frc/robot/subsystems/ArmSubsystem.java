package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {
    CANSparkMax armSubsystem;
    int subArmiD = 5; //change id to correct id later
    public ArmSubsystem() {
        armSubsystem = new CANSparkMax(subArmiD, MotorType.kBrushless);
    }

    public void armSubsystem(double speed) {
        armSubsystem.set(speed);
    }
}
