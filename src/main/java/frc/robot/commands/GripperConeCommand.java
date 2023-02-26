package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GripperConeCommand extends CommandBase {

    ArmSubsystem m_subsystem;
     
     public GripperConeCommand(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
        m_subsystem.grip_speed(-.5);
        //m_subsystem.grip_goto(100);
         
     }
 
     @Override
     public void end(boolean interrupted) {
         m_subsystem.grip_speed(0);
     }
}
