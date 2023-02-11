package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDHMonitor extends SubsystemBase{
    
    PowerDistribution pdh;
    
    public PDHMonitor() {
        pdh = new PowerDistribution();
    }

    public void periodic() {
        // Get the total current of all channels.
        SmartDashboard.putNumber("Total Current", pdh.getTotalCurrent());
        //individual channel currents
        for(int i=0; i < pdh.getNumChannels(); i++) {
            SmartDashboard.putNumber(i + " Current", pdh.getCurrent(i));
        }
    }

    
}
