package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmExtendCommand extends CommandBase {
    ArmSubsystem m_subsystem;

    public ArmExtendCommand(ArmSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }



   @Override
    public void initialize() {
        m_subsystem.setExtendPosition(-.8);
    }

    @Override
    public void execute() {
        m_subsystem.setExtendPosition(-.8);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setExtendSpeed(0);
        //m_subsystem.setArmAngle(m_subsystem.rotateEncoder.getPosition());
    }
}
