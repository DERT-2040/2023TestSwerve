package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableAlign extends CommandBase{
    TurntableSubsystem m_subsystem;
     
     public TurntableAlign(TurntableSubsystem subsystem) {
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
