package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperReleaseCommand extends CommandBase {

    GripperSubsystem m_subsystem;
    DoubleSupplier m_powerInput;
     
     public GripperReleaseCommand(GripperSubsystem subsystem, DoubleSupplier powerInput) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         m_powerInput = powerInput;
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
