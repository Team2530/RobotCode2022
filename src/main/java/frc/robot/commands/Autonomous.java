// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.FeedbackPanel.PanelMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.FeedbackPanel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.kauailabs.navx.frc.AHRS;

public class Autonomous extends CommandBase {

  /** Creates a new Autonomous. */
  AHRS ahrs;
  XboxController xbox;
  DriveTrain driveTrain;
  Intake intake;
  Shooter shooter;
  FeedbackPanel panel;
  
  Timer timer = new Timer();

  public Autonomous(DriveTrain driveTrain, Intake intake, Shooter shooter, AHRS ahrs, XboxController xbox, FeedbackPanel panel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.shooter = shooter;
    this.ahrs = ahrs;
    this.panel = panel;
    timer.start();
  }

  boolean perhaps = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //panel.setDisplayMode(PanelMode.Boot);
    // Robot travels ~1 m/sec forward and backward
    // Competition settings: 1.5m backward, 1m right (1.5 sec backward, 2 sec right)
    SequentialCommandGroup autoVroomVroom = new SequentialCommandGroup(
        // Runs upper chamber and shooter for 2 seconds - shoots starting ball
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, -Constants.intakeSpeed)),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(1, -Constants.intakeSpeed)),
        new InstantCommand(() -> shooter.setShooterSpeed(0.5)),
        new WaitCommand(3),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, 0)),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(1, 0)),
        new InstantCommand(() -> shooter.setShooterSpeed(0)),
        // Drives backward for 1.5 seconds (~1.5 meters)
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.0, 0.2, 0.0)),
        new WaitCommand(1.5),
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.0, 0.0, 0.0)),
        new WaitCommand(0.5),
        // Strafes to the robot's right for 2 seconds (~1 meter) while running lower intake to acquire ball
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, -Constants.intakeSpeed)),
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.2, 0.0, 0.0)),
        new WaitCommand(3),
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.0, 0.0, 0.0)),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, 0)),
        // Rotates the robot to face the goal before driving back into the tarmac (~1.8 meters)
        new InstantCommand(() -> driveTrain.deathBlossom(26)),
        new WaitCommand(0.5),
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.0, -0.2, 0.0)),
        new WaitCommand(1.8),
        new InstantCommand(() -> driveTrain.driveRobotOriented(0.0, 0.0, 0.0)),
        // Shoots the newly-acquired ball
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, -Constants.intakeSpeed)),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(1, -Constants.intakeSpeed)),
        new InstantCommand(() -> shooter.setShooterSpeed(0.5)),
        new WaitCommand(3),
        // Stops the shootage
        new InstantCommand(() -> intake.setIntakeMotorSpeed(0, 0)),
        new InstantCommand(() -> intake.setIntakeMotorSpeed(1, 0)),
        new InstantCommand(() -> shooter.setShooterSpeed(0.0))
    );
    // DO NOT USE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // SequentialCommandGroup testVroomVroom = new SequentialCommandGroup(
    //   // new InstantCommand(() -> intake.setIntakeMotorSpeed(0, -Constants.intakeSpeed)),
    //   // new InstantCommand(() -> intake.setIntakeMotorSpeed(1, -Constants.intakeSpeed)),
    //   // new InstantCommand(() -> shooter.setShooterSpeed(0.5)),
    //   // new WaitCommand(2),
    //   // new InstantCommand(() -> intake.setIntakeMotorSpeed(1, 0)),
    //   // new InstantCommand(() -> intake.setIntakeMotorSpeed(0, 0)),
    //   // new InstantCommand(() -> shooter.setShooterSpeed(0)),
    //   new AutonomousDrive(driveTrain, 2, 2, ahrs),
    //   new WaitCommand(1.5),
    //   new AutonomousDrive(driveTrain, 2, 3, ahrs)
    // );
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
    //panel.setDisplayMode(PanelMode.Status);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return perhaps;
  }
}
