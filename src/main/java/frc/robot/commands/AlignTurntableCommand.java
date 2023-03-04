package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OpenVisionSubsystem;
import frc.robot.subsystems.TurntableSubsystem;

public class AlignTurntableCommand extends CommandBase {
    OpenVisionSubsystem m_subsystem;
     int process_timer = 1;
     boolean m_turntableStatus = false;
     public AlignTurntableCommand(OpenVisionSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
      if (process_timer != 10) {
         process_timer = process_timer + 1;
      } else {
         m_subsystem.ProcessVision();
         m_turntableStatus = m_subsystem.CheckTurntable();
         process_timer = 1;
      }
      if (!m_turntableStatus) {
         TurntableSubsystem.moveTurntable(1);
      }
     }
 
     @Override
     public void end(boolean interrupted) {
        TurntableSubsystem.moveTurntable(0);
     }
}
