package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeInhaleSubsystem;

public class IntakeInhaleCommand extends CommandBase {
    IntakeInhaleSubsystem m_subsystem;
    int automaticOption;
    int selectedOption;
     public IntakeInhaleCommand(IntakeInhaleSubsystem subsystem, int option) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         selectedOption = option;
         automaticOption = 1;
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
      SmartDashboard.putNumber("Automatic Option", automaticOption);
      if (selectedOption == 0) {
      switch(automaticOption) {
         case 1: 
         m_subsystem.inhale(1);
         break;
         case 2:
         m_subsystem.inhale(0.5);
         break;
         case 3:
         m_subsystem.inhale(-1);
         break;
         default:
         break;
      }
   } else {
      switch(selectedOption) {
         case 1: 
         m_subsystem.inhale(1);
         break;
         case 2:
         m_subsystem.inhale(0.5);
         break;
         case 3:
         m_subsystem.inhale(-1);
         break;
         default:
         break;
      }
   }
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.inhale(0);
        if (automaticOption != 3) {
         automaticOption = automaticOption + 1;
      } else {
         automaticOption = 1;
      }
     }
}
