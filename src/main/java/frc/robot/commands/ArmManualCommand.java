package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommand extends CommandBase {
    ArmSubsystem m_subsystem;
    DoubleSupplier armRotate;
    DoubleSupplier armExtend;
    
     
     public ArmManualCommand(ArmSubsystem subsystem, DoubleSupplier armRotate, DoubleSupplier armExtend) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         this.armRotate = armRotate;
         this.armExtend = armExtend;
         
     }

     
     @Override
     public void initialize() {
        
     }
 
     @Override
     public void execute() {
        if(Math.abs(armExtend.getAsDouble()) > .3) {
            if(Math.abs(armRotate.getAsDouble()) > .2) {
                m_subsystem.manualRotateArm(.3 * armRotate.getAsDouble());
            }
            m_subsystem.setExtendSpeed(.4 * armExtend.getAsDouble());
        } else {
            m_subsystem.rotate(.3 * armRotate.getAsDouble());
        }
        
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
     }
}
