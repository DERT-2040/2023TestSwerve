package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OpenVisionSubsystem;
import frc.robot.subsystems.TurntableSubsystem;

public class OpenVisionCommand extends CommandBase {
   OpenVisionSubsystem m_subsystem;
   int process_timer = 1;
   boolean turntable_isAligned;
     public OpenVisionCommand(OpenVisionSubsystem subsystem) {
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
         //m_subsystem.ProcessVision();
         //SmartDashboard.putBoolean("Turntable Pass/Fail", m_subsystem.CheckTurntable());
         process_timer += 1;
      } else {
         m_subsystem.ProcessVision();
         turntable_isAligned = m_subsystem.CheckTurntable();
         SmartDashboard.putBoolean("Turntable Pass/Fail", turntable_isAligned);
         process_timer = 1;
      }
      if (!turntable_isAligned) {
         TurntableSubsystem.moveTurntable(-1);
      }
     }
 
     @Override
     public void end(boolean interrupted) {
     TurntableSubsystem.moveTurntable(0);
   }
}
