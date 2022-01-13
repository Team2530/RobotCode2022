// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ManualShooter extends CommandBase {
  /** Creates a new ManualTurret. */
  Hood m_hood;
  Joystick stick;

  public ManualShooter(Hood m_hood, Joystick stick) {
    this.m_hood = m_hood;
    this.stick = stick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_hood.setTurretPower(stick.getX());
    // m_hood.setHoodPosition(0.4+(0.2*stick.getRawAxis(3)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_hood.setTurretPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
