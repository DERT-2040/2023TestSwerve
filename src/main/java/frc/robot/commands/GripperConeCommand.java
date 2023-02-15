package frc.robot.commands;
import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperConeCommand extends CommandBase {

    GripperSubsystem m_subsystem;
    boolean m_ButtonInput;
     
     public GripperConeCommand(GripperSubsystem subsystem, boolean ButtonInput) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_ButtonInput = ButtonInput;
     }



    @Override
     public void initialize() {
     }
 
     @Override
     public void execute() {
        m_subsystem.gripCone(m_ButtonInput);
         
     }
 
     @Override
     public void end(boolean interrupted) {
         
     }
}
