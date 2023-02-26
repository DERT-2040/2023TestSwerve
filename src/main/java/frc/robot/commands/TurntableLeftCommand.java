package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableLeftCommand extends CommandBase {
     private final TurntableSubsystem m_subsystem;
     private final GenericHID m_gamePad1;
     private final int m_axisIndex;
     public TurntableLeftCommand(TurntableSubsystem subsystem, GenericHID gamePad1, int axisIndex) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_gamePad1 = gamePad1;
         m_axisIndex = axisIndex;
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        m_subsystem.moveTurntable(m_gamePad1.getRawAxis(m_axisIndex) * 0.5);
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.moveTurntable(0);
     }
}
