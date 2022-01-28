// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  DriveTrain driveTrain;
  Timer timer = new Timer();
  double distanceTraveling = 0.0;
  AHRS ahrs = new AHRS();
  double distanceTraveled = 0.0;
  double velocity = ahrs.getVelocityY();
  double timeElapsed = 0;
  String direction = "";
  double forwardBack = 0;
  double leftRight = 0;
  double lastKnownTime = 0;
  double rotation = 0;
  double currentRotation = 0;
  double orginalRotation = 0;

  public AutonomousDrive(DriveTrain driveTrain, double distanceTraveling, String direction, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.distanceTraveling = distanceTraveling;
    this.direction = direction;
    this.rotation = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // whatever direction is input, the robot (should) go that direction;
    if (direction == "forward") {
      forwardBack = 0.5;
    }
    if (direction == "back") {
      forwardBack = -0.5;
    }
    if (direction == "left") {
      leftRight = -0.5;
    }
    if (direction == "right") {
      leftRight = 0.5;
    }
    timer.reset();
    timer.start();
    // Getting orginal rotation of the robot
    orginalRotation = ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check to see if you are rotating or going a cardinal direction
    if(direction != ""){
    driveTrain.singleJoystickDrive(leftRight, forwardBack, 0);
    if (direction == "forward" || direction == "back") {
      velocity = ahrs.getVelocityY();
    } else {
      velocity = ahrs.getVelocityX();
    }

    timeElapsed = timer.get();
    // Distance = Rate * Time
    distanceTraveled = distanceTraveled + Math.abs(velocity * (timeElapsed - lastKnownTime));
    lastKnownTime = timer.get();
    } else {
      // if not going a cardinal direction, do spin
     driveTrain.singleJoystickDrive(0 , 0, Math.sign(rotation) * 0.2 );
      currentRotation = ahrs.getAngle();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.singleJoystickDrive(0, 0, 0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check to see if you are turning or going a cardinal direction
    if(direction != ""){
     if (distanceTraveling < distanceTraveled) {
      return true;
    } else {
      return false;
    }
    } else {
      if(rotation <= currentRotation - originalRotation){
        return true;
      } else {
        return false;
      }
    }
    }
  }