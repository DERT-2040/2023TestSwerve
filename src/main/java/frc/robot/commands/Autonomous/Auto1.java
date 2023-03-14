package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autonomous.Components.AutoSimpleDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveControlSubsystem;

public class Auto1 extends CommandBase {
    DriveControlSubsystem m_subsystem;
    AutoSimpleDrive drive1;

    
     //settings: 0 low, 1 mid, 2 high, 3 full retracted
     public Auto1(DriveControlSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         drive1 = new AutoSimpleDrive(subsystem, 1, 0, 0, true);

         
     }

     
     @Override
     public void initialize() {
        
        
     }
 
     @Override
     public void execute() {
        double time = Timer.getFPGATimestamp();
        if(time < 2) {
            if(!drive1.isScheduled()) {
                drive1.schedule();
            }
        } else {
            if(drive1.isScheduled()) {
                drive1.cancel();
            }
        }
        
     }
 
     @Override
     public void end(boolean interrupted) {
     }

     @Override
    public boolean isFinished() {
        return false;
    }
}