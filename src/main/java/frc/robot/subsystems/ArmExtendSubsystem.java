package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    CANSparkMax extendMotor;
    int extendID = 30;
    public ArmExtendSubsystem() {
        extendMotor = new CANSparkMax(extendID, MotorType.kBrushless);
    }

    public void extend(double speed) {
        extendMotor.set(speed);
    }
}
