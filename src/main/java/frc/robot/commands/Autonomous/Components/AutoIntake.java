package frc.robot.commands.Autonomous.Components;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.IntakeExhaleCommand;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.IntakeInhaleSubsystem;

public class AutoIntake extends CommandBase {
    IntakeExtendSubsystem m_extend;
    IntakeInhaleSubsystem m_inhale;
    int position;
    double speed;
    
    

    /**in: 0, Mid: 20, Out: 47
     * Cone: 1, Cube: 0.5
     */
    public AutoIntake(IntakeExtendSubsystem extend, IntakeInhaleSubsystem inhale, int position, double speed) {
        m_extend = extend;
        m_inhale = inhale;
        
        addRequirements(m_extend);
        this.position = position;
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extend.goToPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extend.periodic();
    m_inhale.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

    
}
