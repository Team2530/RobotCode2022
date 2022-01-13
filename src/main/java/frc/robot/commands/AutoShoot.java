// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Revolver;
import frc.robot.subsystems.Hood;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  Revolver m_revolver;
  Hood m_hood;

  public AutoShoot(Revolver m_revolver, Hood m_hood) {
    this.m_revolver = m_revolver;
    this.m_hood = m_hood;
  }

  boolean hasRotated = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.flywheelRotateSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Hood.motor_flywheel.getMotorOutputPercent() > 0.9 && !hasRotated) {
      m_revolver.rotateRevolver(90);
      hasRotated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hood.flywheelRotateSpeed(0);
    m_revolver.setRevolverSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasRotated;
  }
}
