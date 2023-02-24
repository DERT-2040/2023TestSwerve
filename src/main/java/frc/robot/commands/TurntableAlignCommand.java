package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;
import frc.robot.subsystems.OpenVisionSubsystem;
public class TurntableAlignCommand extends CommandBase{
    TurntableSubsystem m_subsystem;
     
     public TurntableAlignCommand(TurntableSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         
     }

     
     @Override
     public void initialize() {
        OpenVisionSubsystem.ProcessVision();
     }
 
     @Override
     public void execute() {
        OpenVisionSubsystem.ProcessVision();
        SmartDashboard.putBoolean("Turntable Pass or Fail", OpenVisionSubsystem.CheckTurntable());
     }
 
     @Override
     public void end(boolean interrupted) {
     }
}
