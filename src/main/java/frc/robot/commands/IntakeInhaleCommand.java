package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeInhaleSubsystem;

public class IntakeInhaleCommand extends CommandBase {
    IntakeInhaleSubsystem m_subsystem;
     public IntakeInhaleCommand(IntakeInhaleSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
      m_subsystem.inhale();
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.stop();
     }
}
