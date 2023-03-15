package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autonomous.Components.AutoSimpleDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveControlSubsystem;

public class AutoTemplate extends CommandBase {
    DriveControlSubsystem m_subsystem;
    AutoSimpleDrive drive1;
    double startTime;
    
     //settings: 0 low, 1 mid, 2 high, 3 full retracted
     public AutoTemplate(DriveControlSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        drive1 = new AutoSimpleDrive(subsystem, 0, 1, 0, true);
        

         
     }

     
     @Override
     public void initialize() {
    
        startTime = Timer.getFPGATimestamp();
        
        
     }
 
     @Override
     public void execute() {
        double time = Timer.getFPGATimestamp() - startTime;
        
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        if(time < 2) {
            if(!drive1.isScheduled()) {
                drive1.schedule();
            }
        } else if (time < 9) {
            if(drive1.isScheduled()) {
                drive1.cancel();
            }
            //m_subsystem.simpleDrive(0, 0, 0, false);
        }
        
     }
 
     @Override
     public void end(boolean interrupted) {
        SmartDashboard.putNumber("stopped", 1);
     }

    /* @Override
    public boolean isFinished() {
        return false;
    }*/

    public void stopCommand(Command command) {
        if(command.isScheduled()) {
            command.cancel();
        }
    }

    public void startCommand(Command command) {
        if(!command.isScheduled()) {
            command.schedule();
        }
    }
}