package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmReverseCommand extends CommandBase {
    ArmSubsystem m_subsystem;

    public ArmReverseCommand(ArmSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }



   @Override
    public void initialize() {
        m_subsystem.setArmAngle(90);
    }

    @Override
    public void execute() {
        m_subsystem.setArmAngle(90);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setArmSpeed(0);
        m_subsystem.setExtendSpeed(0);
        //m_subsystem.setArmAngle(m_subsystem.rotateEncoder.getPosition());
    }
}
