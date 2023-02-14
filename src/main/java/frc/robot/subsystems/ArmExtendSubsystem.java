package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    CANSparkMax extendMotor;
    int extendID = 5;
    public ArmExtendSubsystem() {
        extendMotor = new CANSparkMax(extendID, MotorType.kBrushless);
    }

    public void xtend(double speed) {
        extendMotor.set(speed);
    }
}
