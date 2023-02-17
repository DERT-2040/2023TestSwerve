package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    Spark inhaleMotor;
    int inhaleID = 2;

    Spark extendMotor;
    int extendID = 3;//21;
    private RelativeEncoder extendEncoder;

    public IntakeSubsystem() {
        inhaleMotor =new Spark(inhaleID);
        extendMotor = new Spark(extendID);//CANSparkMax(extendID, MotorType.kBrushless);
        //extendEncoder = extendMotor.getEncoder();
    }

    //Kirby
    public void inhale(double speed) {
        inhaleMotor.set(speed);
    }

    public void extend(double speed) {
        extendMotor.set(speed);
    }

    /*public double getExtendPosition() {
        return extendEncoder.getPosition();
        
    }*/

}

