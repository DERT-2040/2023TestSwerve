package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OpenVisionSubsystem;

public class OpenVisionCommand extends CommandBase {
   OpenVisionSubsystem m_subsystem;
   int m_mode;
   int process_timer = 1;
     public OpenVisionCommand(OpenVisionSubsystem subsystem, int mode) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_mode = mode;
     }

     
     @Override
     public void initialize() {
      if (process_timer != 10) {
         m_subsystem.ProcessVision();
         SmartDashboard.putBoolean("Turntable Pass/Fail", m_subsystem.CheckTurntable());
      } else {
         m_subsystem.ProcessVision();
         SmartDashboard.putBoolean("Turntable Pass/Fail", m_subsystem.CheckTurntable());
         process_timer = 1;
      }
     }
 
     @Override
     public void execute() {
      
     }
 
     @Override
     public void end(boolean interrupted) {
      switch (m_mode) {
         case 1:
         m_subsystem.ProcessVision();
         SmartDashboard.putBoolean("Turntable Pass/Fail", m_subsystem.CheckTurntable());
         break;
         case 2:
         m_subsystem.ProcessVision();
         SmartDashboard.putNumber("Alignment Output", m_subsystem.CheckRobotAlignment());
         break;
      }
     }
}
