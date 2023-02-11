package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
    VisionSubsystem m_subsystem;
    //Pose2d pose;
     
     public VisionCommand(VisionSubsystem subsystem) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
     }

     
     @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
        //var temp = m_subsystem.getEstimatedGlobalPose());
        //SmartDashboard.putString("visionPose", m_subsystem.getPose().toString());
         
         
     }
 
     @Override
     public void end(boolean interrupted) {
         
     }
}