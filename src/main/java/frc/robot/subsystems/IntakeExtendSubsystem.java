package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtendSubsystem extends SubsystemBase {
    
    CANSparkMax extendMotor;
    int extendID = 21; //3;//21;
    private RelativeEncoder extendEncoder;

    public IntakeExtendSubsystem() {
        extendMotor = new CANSparkMax(extendID, MotorType.kBrushless); //extendMotor = new Spark(extendID);//
        //extendEncoder = extendMotor.getEncoder();
    }

    public void extend(double speed) {
        extendMotor.set(speed);
    }

    /*public double getExtendPosition() {
        return extendEncoder.getPosition();
        
    }*/

}

