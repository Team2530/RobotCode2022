/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  XboxController xboxController = new XboxController(0);
  private int camera = 1;
  private double yawTarget = 0.0;
  PhotonCamera BSideCamera = new PhotonCamera("Breaker");
  // PhotonCamera ESideCamera = new PhotonCamera("Ethernet");
  PhotonCamera curCamera = BSideCamera;
  private final double yawRate = 310.0;
  double forwardSpeed;
  double rotationSpeed;
  double kP = 0.5, kI = 0, kD = 0;
  private double lastExecuted = Timer.getFPGATimestamp();
  PIDController forwardController = new PIDController(kP, kI, kD);
  PIDController turnController = new PIDController(kP, kI, kD);

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

    if (xboxController.getAButtonPressed()) {
      camera = camera * -1;
    }

    if (xboxController.getYButton()) {
      if (camera == 1) {
        curCamera = BSideCamera;
      } else {
        curCamera = BSideCamera;
        ;
      }
      BSideCamera.setPipelineIndex(2);

      // TODO: Make this the other PiD

      var result = curCamera.getLatestResult();
      double gain = 1.0;
      if (result.hasTargets()) {
        System.out.println();
        gain = Math.min(0.75, Math.sqrt(result.getBestTarget().getArea()) / 2);
        rotationSpeed = turnController.calculate(result.getBestTarget().getYaw() / 30, -0.1);
        if (result.getBestTarget().getArea() > 40.0)
          rotationSpeed = 0.0;
        forwardSpeed = xboxController.getLeftX();
      } else {
        System.out.println("No Ball(s)");
        rotationSpeed = 0;
      }

      m_drivetrain.mecanumDrive.driveCartesian(xboxController.getLeftY() / 2, xboxController.getLeftX() / 2,
          rotationSpeed);
    } else {// if' (stick.getMagnitude() < 0.2) return;
      double m = RobotContainer.getBoostMode() ? 1.0 : 0.5;
      double s = RobotContainer.getSlowMode() ? 0.5 : 1;
      // m *= (stick.getRawAxis(3) + 1.0) / 2.0;

      double deltaTime = Timer.getFPGATimestamp() - lastExecuted;
      lastExecuted = Timer.getFPGATimestamp();

      // double turn = stick.getRawAxis(3) - stick.getRawAxis(2);
      yawTarget += stick.getZ() * yawRate * deltaTime;
      m_drivetrain.singleJoystickDrive(Deadzone.deadZone(stick.getRawAxis(1), Constants.deadzone) * m * s,
          Deadzone.deadZone(stick.getRawAxis(0), Constants.deadzone) * m * s,
          Deadzone.deadZone(stick.getRawAxis(2), Constants.deadzoneZ) * m * s);
      // m_drivetrain.singleJoystickDrive(stick.getX() * m, 0, 0);
    }
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
