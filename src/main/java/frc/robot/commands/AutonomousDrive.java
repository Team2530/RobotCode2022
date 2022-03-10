// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  DriveTrain driveTrain = new DriveTrain();
  AHRS ahrs = new AHRS();
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
  public AutonomousDrive(DriveTrain driveTrain, double distance, int direction) {
    this.driveTrain = driveTrain;
    this.direction = direction;
    this.distance = distance;
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
    currentVelocity = navxData[4];
    navxGhostNumbers(navxData);
    // only for testing!
    for(int i = 0; i < 5; i++){
    System.out.print(navxData[i] + " ");
    }
    distanceMaths();
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
    // if (direction == 1 || direction == 2) {
    //   return ahrs.getVelocityX();
    // } else if (direction == 3 || direction == 4) {
    //   return ahrs.getVelocityY();
    // } else {
    //   return 0.0;
    // }
    return Math.random();
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
  public void navxGhostNumbers(double[] navxData) {
    this.navxData = navxData;
    double averageNavxSpeed = 0;
    for(int e = 0; e < 5; e++) {
      averageNavxSpeed = averageNavxSpeed + navxData[e];
    }
    // calculates the average navx speed to eliminate speeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeed
    // yeeeettoooo the speeeeeeeeeeeeeeeeeeeeeeeed
    averageNavxSpeed = averageNavxSpeed / 4;
    if (Math.abs(robotVelocity() - .3) < averageNavxSpeed || 
    Math.abs(robotVelocity() + .3) > averageNavxSpeed) {
      for(int i = 0; i < 5; i++) {
        if(i < 4) {
          navxData[i] = navxData[i+1];
        } else {
          navxData[i] = robotVelocity();
        }
      }
    } else {
      System.out.println("It did the thing");
      navxData[4] = navxData[3];
    }
  } 
}