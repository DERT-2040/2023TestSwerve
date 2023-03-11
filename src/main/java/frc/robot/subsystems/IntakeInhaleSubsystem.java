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
    //True is yellow, false is purple
    boolean selectedObject = true;

    public IntakeInhaleSubsystem() {
        inhaleMotor = new Spark(inhaleID);
    }

    //Kirby
    public void inhale() {
        if (selectedObject) {
            inhaleMotor.set(1);
        } else {
            inhaleMotor.set(0.5);
        }
    }
    public void switchObject() {
        if (selectedObject) {
            selectedObject = false;
        } else {
            selectedObject = true;
        }
    }
    public void stop() {
        inhaleMotor.set(0);
    }
}

