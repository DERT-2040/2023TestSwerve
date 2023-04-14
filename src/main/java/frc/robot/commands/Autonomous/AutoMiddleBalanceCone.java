package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Autonomous.Components.AutoArm;
import frc.robot.commands.Autonomous.Components.AutoBalance;
import frc.robot.commands.Autonomous.Components.AutoGrip;
import frc.robot.commands.Autonomous.Components.AutoSimpleDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveControlSubsystem;

public class AutoMiddleBalanceCone extends CommandBase {
    double startTime;
    
    
    DriveControlSubsystem driveSub;
    ArmSubsystem armSub;

    AutoSimpleDrive drive1;
    AutoSimpleDrive drive2;
    AutoSimpleDrive drive3;
    AutoBalance balance;
    AutoGrip gripCone;
    AutoGrip gripRelease;
    AutoArm armTop;
    AutoArm armRetract;
    
    
    
     //settings: 0 low, 1 mid, 2 high, 3 full retracted
     public AutoMiddleBalanceCone(DriveControlSubsystem drive, ArmSubsystem arm) {
        driveSub = drive;
        armSub = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        drive1 = new AutoSimpleDrive(drive, 0, .15, 0, true);
        drive2 = new AutoSimpleDrive(drive, 0, -1.5, 0, true);
        drive3 = new AutoSimpleDrive(drive, 0, -.5, 0, true);
        balance = new AutoBalance(drive);
        gripCone = new AutoGrip(armSub, .05);
        gripRelease = new AutoGrip(armSub, 1.1);
        armTop = new AutoArm(armSub, 2);
        armRetract = new AutoArm(armSub, 3);
    


        

         
     }

     
     @Override
     public void initialize() {
    
        startTime = Timer.getFPGATimestamp();
        
        
     }
 
     @Override
     public void execute() {
        double time = Timer.getFPGATimestamp() - startTime;
        
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        if(time < 1) {
            /*if(!drive1.isScheduled()) {
                drive1.schedule();
            }*/
            startCommand(gripCone);
        } else if (time < 4.5) {
            /*if(drive1.isScheduled()) {
                drive1.cancel();
            }*/
            stopCommand(gripCone);
            startCommand(armTop);
        } else if (time < 5.5) {
            stopCommand(armTop);
            startCommand(drive1);
        } else if(time < 6.5) {
            startCommand(gripRelease);
            stopCommand(drive1);
        } else if (time < 7.5) {
            stopCommand(gripRelease);
            startCommand(drive2);
        } else if (time < 7.75) {
            startCommand(armRetract);
        } else if (time < 10.5) {
            stopCommand(drive2);
            startCommand(balance);
        } else if( time < 14.5) {
            
            stopCommand(armRetract);
            
        } else if (time < 15) {
            stopCommand(balance);
            
        }
        
     }
 
     @Override
     public void end(boolean interrupted) {
        SmartDashboard.putNumber("stopped", 1);
     }

    /* @Override
    public boolean isFinished() {
        return false;
    }*/

    public void stopCommand(Command command) {
        if(command.isScheduled()) {
            command.cancel();
        }
    }

    public void startCommand(Command command) {
        if(!command.isScheduled()) {
            command.schedule();
        }
    }
}