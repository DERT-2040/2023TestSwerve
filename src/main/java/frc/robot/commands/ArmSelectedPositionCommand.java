package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSelectedPositionCommand extends CommandBase {
    ArmSubsystem m_subsystem;
    IntSupplier selectedPos;
    
     
     public ArmSelectedPositionCommand(ArmSubsystem subsystem, IntSupplier selectedPos) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);

         this.selectedPos = selectedPos;
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.setArmWithSetting(selectedPos.getAsInt());
         //m_subsystem.rotate(.3);
         //m_subsystem.setArmAngle(30);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
     }
}
