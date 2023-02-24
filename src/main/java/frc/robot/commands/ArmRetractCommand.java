package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRetractCommand extends CommandBase {
    ArmSubsystem m_subsystem;

    public ArmRetractCommand(ArmSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }



   @Override
    public void initialize() {
    //    m_subsystem.setExtendPosition(-0.1);
    m_subsystem.setExtendSpeed(0.3);
    }

    @Override
    public void execute() {
    //    m_subsystem.setExtendPosition(-0.1);

        
        
    }

    @Override
    public void end(boolean interrupted) {
    //    m_subsystem.setExtendSpeed(0);

        //m_subsystem.setArmAngle(m_subsystem.rotateEncoder.getPosition());
    }
}
