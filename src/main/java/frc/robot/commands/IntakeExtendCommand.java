package frc.robot.commands;

//import org.opencv.osgi.OpenCVInterface;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtendSubsystem;

public class IntakeExtendCommand extends CommandBase {
    IntakeExtendSubsystem m_subsystem;
    int automaticOption;
    int selectedOption;
     public IntakeExtendCommand(IntakeExtendSubsystem subsystem, int option) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         selectedOption = option;
         automaticOption = 1;
     }

     
     @Override
     public void initialize() {
      if (selectedOption == 0) {
         switch(automaticOption) {
            case 1: 
            m_subsystem.goToPosition(0);
            break;
            case 2:
            m_subsystem.goToPosition(15);
            break;
            case 3:
            m_subsystem.goToPosition(47);
            break;
            default:
            break;
         }
         
      } else {
         switch(selectedOption) {
            case 1: 
            m_subsystem.goToPosition(0);
            break;
            case 2:
            m_subsystem.goToPosition(15);
            break;
            case 3:
            m_subsystem.goToPosition(47);
            break;
            default:
            break;
         }
      }
     }
 
     @Override
     public void execute() {
      //m_subsystem.extend(1.0);
      //SmartDashboard.putNumber("Extend Auto Option", automaticOption);
      
     }
 
     @Override
     public void end(boolean interrupted) {
      m_subsystem.extend(0);
      if (automaticOption != 3) {
         automaticOption = automaticOption + 1;
      } else {
         automaticOption = 1;
      }
     }
}
