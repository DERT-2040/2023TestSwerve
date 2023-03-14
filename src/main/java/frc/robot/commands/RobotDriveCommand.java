package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveControlSubsystem;

public class RobotDriveCommand extends CommandBase {
    DriveControlSubsystem m_subsystem;
    
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rot;
    DoubleSupplier gamePadX;
    DoubleSupplier gamePadY;
    BooleanSupplier boostTrigger;
    BooleanSupplier superSpeed;
    BooleanSupplier lock;
    BooleanSupplier auto;
    BooleanSupplier balancing;
    AHRS ahrs;

    public RobotDriveCommand(DriveControlSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DoubleSupplier gamePadX, DoubleSupplier gamePadY, BooleanSupplier boostTrigger, BooleanSupplier superSpeed, BooleanSupplier lock, BooleanSupplier auto, BooleanSupplier balancing) {
        m_subsystem = subsystem;
        
        addRequirements(m_subsystem);

        this.x = x;
        this.y = y;
        this.rot = rot;
        this.gamePadX = gamePadX;
        this.gamePadY = gamePadY;
        this.boostTrigger = boostTrigger;
        this.superSpeed = superSpeed;
        this.lock = lock;
        this.auto = auto;
        this.balancing = balancing;
    

        // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble(), gamePadX.getAsDouble(), gamePadY.getAsDouble(), boostTrigger.getAsBoolean(), superSpeed.getAsBoolean(), lock.getAsBoolean(), auto.getAsBoolean(), balancing.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
