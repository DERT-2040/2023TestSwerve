package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableRightCommand extends CommandBase {
    TurntableSubsystem m_subsystem;
     double selectedSpeed = 1;
     public TurntableRightCommand(TurntableSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.moveTurntable(selectedSpeed * -0.5);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.moveTurntable(0);
     }
}
