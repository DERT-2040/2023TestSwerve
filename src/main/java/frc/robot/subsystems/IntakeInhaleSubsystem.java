package frc.robot.subsystems;
/*
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
*/
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeInhaleSubsystem extends SubsystemBase {
    Spark inhaleMotor;
    int inhaleID = 2;

    public IntakeInhaleSubsystem() {
        inhaleMotor = new Spark(inhaleID);
    }

    //Kirby
    public void inhale(double speed) {
        inhaleMotor.set(speed);
    }


}

