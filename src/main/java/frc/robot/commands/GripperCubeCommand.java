package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
public class GripperCubeCommand extends CommandBase {

    ArmSubsystem m_subsystem;
    boolean m_ButtonInput;
     
     public GripperCubeCommand(ArmSubsystem subsystem, boolean ButtonInput) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_ButtonInput = ButtonInput;
     }

    @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
         m_subsystem.gripCube(m_ButtonInput);
         
     }
 
     @Override
     public void end(boolean interrupted) {
         
     }
}
