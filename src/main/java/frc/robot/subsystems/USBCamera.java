// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class USBCamera extends CommandBase {
  static HttpCamera driveCam, leftIntakeCam, rightIntakeCam;
  static MjpegServer dashboardCam = CameraServer.addSwitchedCamera("Dashcam");

  /** Creates a new USBCamera. */
  public USBCamera() {
    // Use addRequirements() here to declare subsystem dependencies.
    driveCam = new HttpCamera("driveCam", "http://10.25.30.2:1182/stream.mjpg");
    leftIntakeCam = new HttpCamera("leftIntakeCam", "http://10.25.30.55:1183/stream.mjpg");
    rightIntakeCam = new HttpCamera("rightIntakeCam", "http://10.25.30.55:1184/stream.mjpg");
    dashboardCam.setSource(driveCam);
    Shuffleboard.getTab("Driver Dashboard")
        .add("Camera", dashboardCam.getSource())
        .withWidget(BuiltInWidgets.kCameraStream)
        .withSize(4, 4)
        .withPosition(7, 0);
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
    if (mode == DriveTrain.Cockpit.LEFT) {
      putToDashboard(leftIntakeCam);
    } else if (mode == DriveTrain.Cockpit.RIGHT) {
      putToDashboard(rightIntakeCam);
    } else {
      putToDashboard(driveCam);
    }
  }

  private static void putToDashboard(HttpCamera source) {
    dashboardCam.setSource(source);
  }
}
