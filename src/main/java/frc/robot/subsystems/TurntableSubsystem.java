package frc.robot.subsystems;
/*
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
*/
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurntableSubsystem extends SubsystemBase {
    static Talon turntableMotor;
    int turntableMotorID = 3;

    public TurntableSubsystem() {
        turntableMotor = new Talon(turntableMotorID);
    }

    //WE SPIN WAAAAAAAHHHHHHHHHOOOOOOOOOOOOOOOOOOOO
    public static void moveTurntable(double speed) {
        turntableMotor.set(speed);
    }


}

