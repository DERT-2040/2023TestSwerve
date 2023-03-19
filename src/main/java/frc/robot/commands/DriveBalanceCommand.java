package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveControlSubsystem;

public class DriveBalanceCommand extends CommandBase {
    
    DriveControlSubsystem m_subsystem;
        
        public DriveBalanceCommand(DriveControlSubsystem subsystem) {
            m_subsystem = subsystem;
            // Use addRequirements() here to declare subsystem dependencies.
            addRequirements(subsystem);
            
        }

        
        @Override
        public void initialize() {
        
        }
    
        @Override
        public void execute() {
        m_subsystem.balance();
        }
    
        @Override
        public void end(boolean interrupted) {
        m_subsystem.simpleDrive(0, 0, 0, true);
        }
}
