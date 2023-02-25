package frc.robot.commands;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;

public class IntakeExtendCommand extends CommandBase {
    IntakeExtendSubsystem m_subsystem;
    int automaticOption;
    int selectedOption = 1;
     public IntakeExtendCommand(IntakeExtendSubsystem subsystem, int option) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         selectedOption = option;
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
      if (selectedOption == 0) {
      switch(automaticOption) {
         case 1: 
         m_subsystem.goToPosition(10);
         break;
         case 2:
         m_subsystem.goToPosition(20);
         break;
         case 3:
         m_subsystem.goToPosition(30);
         break;
         default:
         break;
      }
      if (automaticOption != 3) {
         automaticOption =+ 1;
      } else if (automaticOption == 3) {
         automaticOption = 1;
      }
   } else {
      switch(selectedOption) {
         case 1: 
         m_subsystem.goToPosition(10);
         break;
         case 2:
         m_subsystem.goToPosition(20);
         break;
         case 3:
         m_subsystem.goToPosition(30);
         break;
         default:
         break;
      }
   }
     }
 
     @Override
     public void end(boolean interrupted) {
     }
}
