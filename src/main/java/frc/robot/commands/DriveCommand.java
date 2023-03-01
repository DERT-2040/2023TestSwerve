// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperVisoryDrive;



public class DriveCommand extends CommandBase {

    SuperVisoryDrive m_subsystem;
    DoubleSupplier drvX;
    DoubleSupplier drvY;
    DoubleSupplier drvRot;
    BooleanSupplier drvAuto;

    
    double m_stickX;                 // driver input x request
    double m_stickY;                 // driver input Y request
    double m_rotate;                 // drive rotate request
    boolean m_auto;                  // auto drive to selected location

  /** Creates a new DriveCommand. */
    public DriveCommand(SuperVisoryDrive subsystem, DoubleSupplier drvX, DoubleSupplier drvY, DoubleSupplier drvRot, BooleanSupplier drvAuto) {
   



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.drvX = drvX;
    this.drvY = drvY;
    this.drvRot = drvRot;
    this.drvAuto = drvAuto;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_stickX = drvX.getAsDouble();
    m_stickY = drvY.getAsDouble();
    m_rotate = drvRot.getAsDouble();
    m_auto   = drvAuto.getAsBoolean();

    m_subsystem.drv(m_stickX, m_stickY, m_rotate, m_auto);
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
