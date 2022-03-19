// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  AHRS ahrs = new AHRS();
  DriveTrain driveTrain = new DriveTrain(ahrs);
  Timer timer = new Timer();
  int direction = 0;
  double distance = 0;
  double currentVelocity = 0.0;
  double timeSinceChecked = 0.0;
  double deltaTime = 0.0;
  double distanceTraveled = 0.0;
  double Dtime = 0.0;
  double[] navxData = {0, 0, 0, 0, 0};

  /**
   * 1 is forward, 2 is back, 3 is right, 4 is left
   */
  public AutonomousDrive(DriveTrain driveTrain, double distance, int direction, AHRS ahrs) {
    this.driveTrain = driveTrain;
    this.direction = direction;
    this.distance = distance;
    this.ahrs = ahrs;
    currentVelocity = 0.0;
    distanceTraveled = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // figure out which direction to go (1 is forward) (2 is back) (3 is right) (4
    // is left)
    ahrs.reset();
    distanceTraveled = 0;
    if (direction == 1) {
      driveTrain.singleJoystickDrive(0.0, 0.2, 0.0);
    }
    if (direction == 2) {
      driveTrain.singleJoystickDrive(0.0, -0.2, 0.0);
    }
    if (direction == 3) {
      driveTrain.singleJoystickDrive(0.2, 0.0, 0.0);
    }
    if (direction == 4) {
      driveTrain.singleJoystickDrive(-0.2, 0.0, 0.0);
    }
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robotVelocity() < Constants.maxVelocityMetersPerSecond){
      currentVelocity = robotVelocity();
     distanceMaths();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.singleJoystickDrive(0.0, 0.0, 0.0);
    //throw new Error("You're done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCondition();
  }

  public double robotVelocity() {
    if (direction == 1 || direction == 2) {
      return ahrs.getVelocityX();
    } else if (direction == 3 || direction == 4) {
      return ahrs.getVelocityY();
    } else {
      return 0.0;
    }
  }

  public void distanceMaths() {
    deltaTime = timer.get() - timeSinceChecked;
    timeSinceChecked = timer.get();
    distanceTraveled = distanceTraveled + Math.abs((currentVelocity * deltaTime));
    System.out.println("distance to go : " + (distance - distanceTraveled)+ " Dtime :    " + deltaTime);
  }

  public boolean endCondition() {
    return distanceTraveled >= distance;
  }
  /**
   * fixes the numbers that are saying the robot is going the speed of light^2
   */
}