// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmExtendCommand extends CommandBase {

  ArmSubsystem m_subsystem;
  /** Creates a new ArmExtendCommand. */
  public ArmExtendCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

        //    m_subsystem.setExtendPosition(-0.1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_subsystem.setExtendSpeed(-0.1);
    m_subsystem.setExtendPosition(.85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_subsystem.setExtendSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
