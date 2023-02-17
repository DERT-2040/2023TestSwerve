package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInhaleCommand extends CommandBase {
    IntakeSubsystem m_subsystem;
     
     public IntakeInhaleCommand(IntakeSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.inhale(0.1);
     }
 
     @Override
     public void end(boolean interrupted) {

     }
}
