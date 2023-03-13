package frc.robot.commands.Autonomous;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoGripRelease extends CommandBase {
    ArmSubsystem m_subsystem;
    
     
     public AutoGripRelease(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.grip_goto(1.1);
        
         //m_subsystem.rotate(.3);
         //m_subsystem.setArmAngle(30);
     }
 
     @Override
     public void end(boolean interrupted) {
        
        m_subsystem.grip_speed(0);
     }

     @Override
    public boolean isFinished() {
        return (m_subsystem.gripInPosition);
    }
}
