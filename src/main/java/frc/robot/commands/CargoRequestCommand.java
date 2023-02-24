// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.RobotContainer;

public class CargoRequestCommand extends CommandBase {

  LEDSubsystem m_LEDSubsystem;    

  /** Creates a new CargoRequestCommand. */
  public CargoRequestCommand(LEDSubsystem subsystem) {
    m_LEDSubsystem = subsystem;
    addRequirements(m_LEDSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDSubsystem.setColor(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
