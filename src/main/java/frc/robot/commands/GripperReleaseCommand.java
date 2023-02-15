package frc.robot.commands;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperReleaseCommand extends CommandBase {

    GripperSubsystem m_subsystem;
    Boolean m_ButtonInput;
     
     public GripperReleaseCommand(GripperSubsystem subsystem, Object ButtonInput) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_ButtonInput = (Boolean)ButtonInput;
     }



@Override
     public void initialize() {
        m_subsystem.gripRelease();
     }
 
     @Override
     public void execute() {
        m_subsystem.gripRelease();
         
     }
 
     @Override
     public void end(boolean interrupted) {
         
     }
}
