package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurntableSubsystem extends SubsystemBase{
    Talon turntable_talon;
    public TurntableSubsystem() {
        turntable_talon = new Talon(3);
    }

    public void turntableSpeed (double power) {
        turntable_talon.set(power);
        SmartDashboard.putNumber("Turntable Power", power);
    }
}
