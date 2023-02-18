package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;

public class IntakeRetractCommand extends CommandBase {
    IntakeExtendSubsystem m_subsystem;
     
     public IntakeRetractCommand(IntakeExtendSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.extend(-.3);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.extend(0);
     }
}
