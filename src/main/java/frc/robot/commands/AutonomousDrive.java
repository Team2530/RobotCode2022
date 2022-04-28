// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.libraries.Deadzone;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.schedulers.SequentialScheduler;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  AHRS ahrs;
  Timer timer = new Timer();
  DriveTrain driveTrain;
  int direction = 0;
  double distance = 0;
  double currentVelocity = 0.0;
  double timeSinceChecked = 0.0;
  double deltaTime = 0.0;
  double distanceTraveled = 0.0;
  double Dtime = 0.0;
  double[] navxData = { 0, 0, 0, 0, 0 };
  double number = 0.0;

  /**
   * @param direction 1 is forward, 2 is back, 3 is right, 4 is left
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

    // don't question it
    // distance = distance / Constants.dontQuestionIt;
    distance = distance - (distance * .15);
    distanceTraveled = 0;

    if (direction == 1) {
      driveTrain.driveRobotOriented(0.0, 0.2, 0.0);
    }
    if (direction == 2) {
      driveTrain.driveRobotOriented(0.0, -0.2, 0.0);
    }
    if (direction == 3) {
      driveTrain.driveRobotOriented(0.2, 0.0, 0.0);
    }
    if (direction == 4) {
      driveTrain.driveRobotOriented(-0.2, 0.0, 0.0);
    }
    timeSinceChecked = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotVelocity();
    if (Math.abs(currentVelocity) < Constants.maxMetersPerSecondForwards) {
      distanceMaths();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finnyeeto");
    driveTrain.driveRobotOriented(0.0, 0.0, 0.0);
    // throw new Error("You're done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCondition();
  }

  public void robotVelocity() {
    deltaTime = Timer.getFPGATimestamp() - timeSinceChecked;
    timeSinceChecked = Timer.getFPGATimestamp();
    if (direction == 1) {
      currentVelocity = ahrs.getWorldLinearAccelY() / 9.8;
      // currentVelocity = Deadzone.deadZone(ahrs.getVelocityY(), 0.05);
    } else if (direction == 2) {
      currentVelocity = ahrs.getWorldLinearAccelY() / 9.8 * -1;
    } else if (direction == 3) {
      currentVelocity = ahrs.getWorldLinearAccelX() / 9.8 * -1;
    } else if (direction == 4) {
      currentVelocity = ahrs.getWorldLinearAccelX() / 9.8;
    } else {
      System.out.println("ERROR");
    }
  }

  public void distanceMaths() {
    distanceTraveled = distanceTraveled + Math.abs(currentVelocity);
    System.out.println("distance to go : " + (distance - distanceTraveled) + " Dtime :    " + deltaTime);
  }

  public boolean endCondition() {
    return distanceTraveled >= distance;
  }
}