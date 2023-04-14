package frc.robot.commands.Autonomous.Components;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveControlSubsystem;

public class AutoDriveToAngle extends CommandBase {
    DriveControlSubsystem m_subsystem;
    
    double x;
    double y;
    double angle;
    boolean fieldRelative;

    /**Positive X is left, Positive Y is backwards*/
    public AutoDriveToAngle(DriveControlSubsystem subsystem, double x, double y, double angle, boolean fieldRelative) {
        m_subsystem = subsystem;
        
        addRequirements(m_subsystem);

        this.x = x;
        this.y = y;
        this.angle = angle;
        this.fieldRelative = fieldRelative;
        
    

        // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.simpleDriveToRotation(x, y, angle, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.simpleDrive(0, 0, 0, fieldRelative);
  }

    
}
