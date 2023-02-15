package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCubeCommand extends CommandBase {

    GripperSubsystem m_subsystem;
     
     public GripperCubeCommand(GripperSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
         m_subsystem.gripCube();
         
     }
 
     @Override
     public void end(boolean interrupted) {
         
     }
}
