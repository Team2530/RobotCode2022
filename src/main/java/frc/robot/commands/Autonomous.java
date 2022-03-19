// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.kauailabs.navx.frc.AHRS;

public class Autonomous extends CommandBase {
  
  /** Creates a new Autonomous. */
  AHRS ahrs = new AHRS();
  DriveTrain driveTrain = new DriveTrain();
  Intake intake = new Intake();
  Timer timer = new Timer();

  public Autonomous(DriveTrain driveTrain, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.intake = intake;
    timer.start();
  }

  boolean perhaps = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AHRS ahrs = new AHRS();
    ahrs.reset();
    SequentialCommandGroup autoVroomVroom = new SequentialCommandGroup(
      // new InstantCommand(() -> intake.setIntakeMotorSpeed(1, 0.85)),
      new WaitCommand(3),
      // new InstantCommand(() -> intake.setIntakeMotorSpeed(1, 0)),
      new AutonomousDrive(driveTrain, 3, 1, ahrs) 
    );
    System.out.println("Starting Autonomous Commands...");
    System.out.println("Please don't run into something!");
    autoVroomVroom.schedule();
   
    // add more commands here
    // autoVroomVroom.schedule();
    // System.out.println("Scheduled!");
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
    return perhaps;
  }
}
