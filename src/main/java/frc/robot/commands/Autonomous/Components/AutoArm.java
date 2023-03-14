package frc.robot.commands.Autonomous.Components;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArm extends CommandBase {
    ArmSubsystem m_subsystem;
    int setting;
    
     //settings: 0 low, 1 mid, 2 high, 3 full retracted
     public AutoArm(ArmSubsystem subsystem, int setting) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         this.setting = setting;
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.setArmWithSetting(setting);
        
        
         //m_subsystem.rotate(.3);
         //m_subsystem.setArmAngle(30);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
     }

     @Override
    public boolean isFinished() {
        return (m_subsystem.armInPosition && m_subsystem.extendInPosition);
    }
}
