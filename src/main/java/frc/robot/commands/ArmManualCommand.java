package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommand extends CommandBase {
    ArmSubsystem m_subsystem;
    DoubleSupplier armRotate;
    DoubleSupplier armExtend;
    BooleanSupplier manualMode;
    DoubleSupplier fineArm;
    
     
     public ArmManualCommand(ArmSubsystem subsystem, DoubleSupplier armRotate, DoubleSupplier armExtend/*, BooleanSupplier manualMode*/) {
         m_subsystem = subsystem;
         // Use addRequirements() here to declare subsystem dependencies.
         addRequirements(subsystem);
         this.armRotate = armRotate;
         this.armExtend = armExtend;
         this.manualMode = manualMode;
         
     }

     
     @Override
     public void initialize() {
        m_subsystem.setSetting(2);
     }
 
     @Override
     public void execute() {
        /*if(!manualMode.getAsBoolean()) {
            if(Math.abs(armExtend.getAsDouble()) > .3) {
                if(Math.abs(armRotate.getAsDouble()) > .2) {
                    m_subsystem.manualRotateArm(.7 * armRotate.getAsDouble());
                }
                m_subsystem.setExtendSpeed(-.4 * armExtend.getAsDouble());
            } else {
                m_subsystem.rotate(-.5 * armRotate.getAsDouble());
            }
        } else {*/
            if(Math.abs(armExtend.getAsDouble()) > .1) {
                
                m_subsystem.setExtendSpeed(-.4 * (armExtend.getAsDouble() - .1));
            }
            
            if(Math.abs(armRotate.getAsDouble()) > .1) {
                m_subsystem.manualRotateArm(.7 * (armRotate.getAsDouble() - .1));
            }
        //}
        
     }
 
     @Override
     public void end(boolean interrupted) {
        m_subsystem.rotate(0);
        m_subsystem.setExtendSpeed(0);
     }
}
