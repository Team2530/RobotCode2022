/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libraries.Deadzone;
import frc.robot.subsystems.DriveTrain;

public class SingleJoystickDrive extends CommandBase {
  /**
   * Creates a new SingleJoystickDrive.
   */
  DriveTrain m_drivetrain;
  Joystick stick;
  private double yawTarget = 0.0;

  private final double yawRate = 310.0;
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
    // m_drivetrain.setBrakeMode(stick.getRawButton(2));

    // m_drivetrain.setCoast(stick.getRawButton(2) ? NeutralMode.Coast :
    // NeutralMode.Brake);

    // if' (stick.getMagnitude() < 0.2) return;
    double m = stick.getRawButton(1) ? 1.0 : 0.5;
    //m *= (stick.getRawAxis(3) + 1.0) / 2.0;

    double deltaTime = Timer.getFPGATimestamp() - lastExecuted;
    lastExecuted = Timer.getFPGATimestamp();

    // double turn = stick.getRawAxis(3) - stick.getRawAxis(2);
    yawTarget += stick.getZ() * yawRate * deltaTime;
    m_drivetrain.singleJoystickDrive(stick.getX() * m, stick.getY() * m, yawTarget);
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
