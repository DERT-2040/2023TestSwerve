package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;
import frc.robot.subsystems.OpenCV_VisionSubsystem;;

public class TurntableAlignCommand extends CommandBase{
    TurntableSubsystem m_subsystem;
     
     public TurntableAlignCommand(TurntableSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
     }
 
     @Override
     public void end(boolean interrupted) {
     }
}
