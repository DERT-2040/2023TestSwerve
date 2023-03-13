package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmTop extends CommandBase {
    ArmSubsystem m_subsystem;
    
     
     public AutoArmTop(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.setArmWithSetting(2);
        m_subsystem.grip_goto(0);
        
         //m_subsystem.rotate(.3);
         //m_subsystem.setArmAngle(30);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
        m_subsystem.grip_speed(0);
     }

     @Override
    public boolean isFinished() {
        return (m_subsystem.armInPosition && m_subsystem.extendInPosition);
    }
}
