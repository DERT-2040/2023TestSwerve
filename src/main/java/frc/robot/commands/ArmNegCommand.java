package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmNegCommand extends CommandBase{
    ArmSubsystem m_subsystem;
    
     
     public ArmNegCommand(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
         m_subsystem.manualRotateArm(.3);
         //m_subsystem.setArmAngle(30);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
     }
}
