// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AutonomousDrive;
import frc.robot.subsystems.DriveTrain;
public class Autonomous extends CommandBase {
  /** Creates a new Autonomous. */
  DriveTrain driveTrain = new DriveTrain();

  public Autonomous(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;

  }

  boolean perhaps = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AutonomousDrive autoVroomVroom = new AutonomousDrive(driveTrain, 1.5, 1);
    autoVroomVroom.schedule();




    // at end of all autonomous, sToP
    perhaps = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return perhaps;
  }
}
