// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class LimeLight extends CommandBase {
  /** Creates a new Limelight. */
  /** X offset from target */
  double tx;
  /** Y offset from target */
  double ty;
  /** Target Area */
  double ta;
  /** Any targets? */
  double tv;

  double turnRate;

  double limekP = 0.3;

  double minCommand = 0.05;

  NetworkTable table;
  // not sure if this is right or not
  public int lightMode = 3;

  int cameraMode = 0;

  DriveTrain driveTrain;

  public LimeLight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    table = NetworkTableInstance.getDefault().getTable("limelight");
    this.driveTrain = driveTrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateValues();
    double distanceFromTarget = distanceToTarget();
    SmartDashboard.putNumber("Lime Distance", distanceFromTarget);

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

  /**
   * Updates the LimeLight's values
   */
  public void updateValues() {
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    tv = table.getEntry("tv").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
  }

  public double toRadians(double input) {
    return input * (Math.PI / 180.0);
  }

  public double distanceToTarget() {
    double r = toRadians(Constants.limeAngle + ty);
    return (Constants.goalHeight - Constants.limeHeight) / Math.tan(r);
  }

  public void changeMode() {
    if (lightMode == 3) {
      lightMode = 1;
    } else {
      lightMode = 3;
    }
  }

  /**
   * Assume that there is a valid target, we will turn to aim at it
   */
  public void aimAtTarget() {
    double e = -tx;

    if (tx < 1) {
      turnRate = limekP * e + minCommand;
    } else {
      turnRate = limekP * e - minCommand;
    }
    // Use this method to turn to robot at the speeds
    driveTrain.turn(turnRate, -turnRate);

  }
}
