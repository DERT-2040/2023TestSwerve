package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeExtendCommand extends CommandBase {
    IntakeSubsystem m_subsystem;
     
     public IntakeExtendCommand(IntakeSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.extend(.1);
     }
 
     @Override
     public void end(boolean interrupted) {

     }
}
