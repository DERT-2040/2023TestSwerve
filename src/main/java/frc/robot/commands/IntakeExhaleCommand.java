package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeInhaleSubsystem;

public class IntakeExhaleCommand extends CommandBase {
    IntakeInhaleSubsystem m_subsystem;
     
     public IntakeExhaleCommand(IntakeInhaleSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.inhale(-1);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.inhale(0);
     }
}
