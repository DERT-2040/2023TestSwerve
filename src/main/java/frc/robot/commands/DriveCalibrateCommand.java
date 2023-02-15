package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCalibrateCommand extends CommandBase {
    //command for climbing
    private DriveSubsystem m_subsystem;
    //public static double climberSpeed = 0;

    
    public DriveCalibrateCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        Preferences.setDouble(m_subsystem.m_frontLeft.GetTurnOffsetKey(), m_subsystem.m_frontLeft.GetTurningCalibrationValue());
        Preferences.setDouble(m_subsystem.m_frontRight.GetTurnOffsetKey(), m_subsystem.m_frontRight.GetTurningCalibrationValue());
        Preferences.setDouble(m_subsystem.m_rearLeft.GetTurnOffsetKey(), m_subsystem.m_rearLeft.GetTurningCalibrationValue());
        Preferences.setDouble(m_subsystem.m_rearRight.GetTurnOffsetKey(), m_subsystem.m_rearRight.GetTurningCalibrationValue());
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}