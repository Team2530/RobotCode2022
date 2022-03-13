/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.subsystems.DriveTrain;

public class SingleJoystickDrive extends CommandBase {
  /**
   * Creates a new SingleJoystickDrive.
   */
  DriveTrain m_drivetrain;
  Joystick stick;

  private double lastExecuted = Timer.getFPGATimestamp();

  public SingleJoystickDrive(DriveTrain m_drivetrain, Joystick stick) {
    this.m_drivetrain = m_drivetrain;
    this.stick = stick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.reset();
    lastExecuted = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if' (stick.getMagnitude() < 0.2) return;
    double m = RobotContainer.getBoostMode() ? 1.0 : 0.5;
    double s = RobotContainer.getSlowMode() ? 0.5 : 1;
    // m *= (stick.getRawAxis(3) + 1.0) / 2.0;

    // double turn = stick.getRawAxis(3) - stick.getRawAxis(2);
    m_drivetrain.singleJoystickDrive(Deadzone.deadZone(stick.getRawAxis(1), Constants.deadzone) * m * s,
        Deadzone.deadZone(stick.getRawAxis(0), Constants.deadzone) * m * s,
        Deadzone.deadZone(stick.getRawAxis(2), Constants.deadzoneZ) * m * s);
    // m_drivetrain.singleJoystickDrive(stick.getX() * m, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
