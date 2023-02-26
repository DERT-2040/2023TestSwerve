// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final boolean m_isYellow;
  public LEDCommand(LEDSubsystem subsystem, boolean isYellow) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    m_isYellow = isYellow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setColor(m_isYellow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
}
