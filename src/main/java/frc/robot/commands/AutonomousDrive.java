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
  AHRS ahrs = new AHRS();
  Timer timer = new Timer();
  int speed = 0;
  int direction = 0;
  double distance = 0;
  double currentVelocity = 0.0;
  double timeSinceChecked = 0.0;
  double deltaTime = 0.0;
  double distanceTraveled = 0.0;
  public AutonomousDrive(DriveTrain driveTrain, double distance, int direction) {
    this.direction = direction;
    this.distance = distance;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
     // figure out which direction to go   (1 is forward) (2 is back) (3 is right) (4 is left)
    timer.reset();
    timer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    currentVelocity = robotVelocity();
    distanceMaths();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCondition();
  }


public double robotVelocity() {
  if(direction == 1 || direction ==2) {
    return ahrs.getVelocityY();
  } else {
    return ahrs.getVelocityX();
  }
}
public void distanceMaths() {
  deltaTime = Timer.getFPGATimestamp() - timeSinceChecked;
  timeSinceChecked = Timer.getFPGATimestamp();
  distanceTraveled = distanceTraveled + (currentVelocity * deltaTime);
}

public boolean endCondition(){
  if(distanceTraveled >= distance){
    return true;
  } else {
    return false;
  }

}

}
