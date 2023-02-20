package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GripPower extends CommandBase {
    ArmSubsystem m_subsystem;
    double direction;
     public GripPower(ArmSubsystem subsystem, double direction) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         this.direction = direction;
     }



    @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
        m_subsystem.grip_speed(direction);
         
     }
 
     @Override
     public void end(boolean interrupted) {
         m_subsystem.grip_speed(0);
     }
}
