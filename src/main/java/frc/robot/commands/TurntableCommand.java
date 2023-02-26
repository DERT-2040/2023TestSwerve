package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class TurntableCommand extends CommandBase {
     private final TurntableSubsystem m_subsystem;
     private final GenericHID m_gamePad1;
     private final int m_axisIndex;
     private int m_mode;
     public TurntableCommand(TurntableSubsystem subsystem, GenericHID gamePad1, int axisIndex, int mode) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_gamePad1 = gamePad1;
         m_axisIndex = axisIndex;
         mode = m_mode;
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
      double powerInput = m_gamePad1.getRawAxis(m_axisIndex) * 2;
      if (powerInput > 1) {
         powerInput = 1;
      }
      switch (m_mode) {
         case 1:
            m_subsystem.moveTurntable(-1 * powerInput);
            break;
         case 2:
            m_subsystem.moveTurntable(powerInput);
            break;
      }
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.moveTurntable(0);
     }
}
