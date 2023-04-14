package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GripperReleaseCommand extends CommandBase {

   ArmSubsystem m_subsystem;
     
     public GripperReleaseCommand(ArmSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
      
     }
 
     @Override
     public void execute() {
      //m_subsystem.grip_speed(.9);
      m_subsystem.grip_goto(1.5);
         
         
     }
 
     @Override
     public void end(boolean interrupted) {
      m_subsystem.grip_speed(0);
     }
}
