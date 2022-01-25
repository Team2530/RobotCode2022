// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotContainer;
import edu.wpi.first.math.trajectory.Trajectory.State;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  DriveTrain driveTrain; 
  AHRS ahrs = new AHRS();
  Timer timer;
  }
  public AutonomousDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.singleJoystickDrive(0.5, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  // distance should be in meters? Needs to be checked
  public void driveForwardAutonomous(double distance){
    double distanceTraveled = 0.0;
    double velocity = ahrs.getVelocityY();
    timer.reset();
    timer.start();
    double time = 0
    while(distance > distanceTraveled){
       driveTrain.singleJoystickDrive(0, 0.5, 0);
       velocity = ahrs.getVelocityY();
       timeElapsed = timer.get();
       // Distance = Rate * Time
       distanceTraveled = distanceTraveled + (velocity * timeElapsed);
       // reset and start time for next loop (There may be a better way to do this)
       timer.reset();
       timer.start();
    }
    // stop motors
    driveTrain.singleJoystickDrive(0, 0 ,0);
  }
}
