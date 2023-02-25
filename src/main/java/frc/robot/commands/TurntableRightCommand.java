package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableRightCommand extends CommandBase {
    TurntableSubsystem m_subsystem;
     double selectedSpeed;
     public TurntableRightCommand(TurntableSubsystem subsystem, double speed) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         selectedSpeed = speed;
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.moveTurntable(-1 * selectedSpeed);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.moveTurntable(0);
     }
}
