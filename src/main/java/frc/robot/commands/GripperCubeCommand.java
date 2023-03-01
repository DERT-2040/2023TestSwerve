package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GripperCubeCommand extends CommandBase {

    ArmSubsystem m_subsystem;
     
     public GripperCubeCommand(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
         m_subsystem.grip_goto(0.52);
        //m_subsystem.grip_speed(.5);
         
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.grip_speed(0);
     }
}
