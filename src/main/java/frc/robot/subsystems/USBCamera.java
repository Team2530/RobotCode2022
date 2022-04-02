// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class USBCamera extends CommandBase {
  static UsbCamera driveCam, leftIntakeCam, rightIntakeCam;

  /** Creates a new USBCamera. */
  public USBCamera() {
    // Use addRequirements() here to declare subsystem dependencies.
    driveCam = CameraServer.startAutomaticCapture("driveCam", 0);
    leftIntakeCam = CameraServer.startAutomaticCapture("leftIntakeCam", 1);
    rightIntakeCam = CameraServer.startAutomaticCapture("rightIntakeCam", 2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static void changeCameraSource(DriveTrain.Cockpit mode) {
    if (mode == DriveTrain.Cockpit.FRONT) {
      putToDashboard(driveCam);
    } else if (mode == DriveTrain.Cockpit.LEFT) {
      putToDashboard(leftIntakeCam);
    } else if (mode == DriveTrain.Cockpit.RIGHT) {
      putToDashboard(rightIntakeCam);
    }
  }

  private static void putToDashboard(UsbCamera source) {
    Shuffleboard.getTab("SmartDashboard")
        .add("Camera", source)
        .withWidget(BuiltInWidgets.kCameraStream);
  }
}
