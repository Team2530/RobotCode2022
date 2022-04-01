/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // -------------------- Joysticks and Buttons -------------------- \\
  // Joysticks
  final Joystick stick1 = new Joystick(Constants.stickport1); // Creates a joystick on port 1

  // Xbox Controller
  final XboxController xbox = new XboxController(Constants.xboxport);

  // The robot's subsystems and commands are defined here...

  // -------------------- Subsystems -------------------- \\

  private final AHRS m_ahrs = new AHRS();
  private final Battery m_battery = new Battery(m_ahrs, xbox);
  private final DriveTrain m_driveTrain = new DriveTrain(m_ahrs);
  private final Climber m_climber = new Climber();
  // private final USBCamera usbCamera = new USBCamera();
  private final PhotonVision vision = new PhotonVision();
  private final Intake intake = new Intake();
  private final Chambers ballDetection = new Chambers(3);
  private final Indicators lights = new Indicators(3);
  private final Shooter shooter = new Shooter();

  // -------------------- Autonomous Commands -------------------- \\
  // insert autonomous commands here

  // -------------------- Telop Commands -------------------- \\
  // insert teleop commands here

  // -------------------- Global Toggles -------------------- \\
  /** Whether or not autonomous/smart systems are disabled. */
  private static boolean manualMode = false;
  private static boolean boostMode = false;
  private static boolean slowMode = false;
  private static boolean manualModeOp = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Button for driving at full speed
    new JoystickButton(stick1, 1).whenPressed(() -> {
      boostMode = true;
    }).whenReleased(() -> {
      boostMode = false;
    });

    // Button for driving slowly
    new JoystickButton(stick1, 2).whenPressed(() -> {
      slowMode = true;
    }).whenReleased(() -> {
      slowMode = false;
    });

    // Disable rotation (not yet in this branch)

    // Zero navX rotation
    new JoystickButton(stick1, 4).whenPressed(() -> m_driveTrain.reset());

    // Velocity retention (not yet in this branch)

    // Button for disabling autonomous/smart functions
    new JoystickButton(stick1, 6).whenPressed(() -> {
      manualMode = true;
    }).whenReleased(() -> {
      manualMode = false;
    });

    // Climber control (RB for full power and LB for low power)
    new JoystickButton(xbox, 6).whenPressed(() -> m_climber.setClimberSpeed(1.0))
        .whenReleased(() -> m_climber.setClimberSpeed(0));
    new JoystickButton(xbox, 5).whenPressed(() -> m_climber.setClimberSpeed(0.3))
        .whenReleased(() -> m_climber.setClimberSpeed(0));

    // Lower intake up (A button)
    new JoystickButton(xbox, 1).whenPressed(() -> intake.setIntakeMotorSpeed(0, -Constants.intakeSpeed))
        .whenReleased(() -> intake.setIntakeMotorSpeed(0, 0));

    // Upper intake up and also shooter (X button)
    new JoystickButton(xbox, 3).whenPressed(
        new ParallelCommandGroup(
            new InstantCommand(() -> {
              intake.setIntakeMotorSpeed(1, -Constants.intakeSpeed);
            }),
            new InstantCommand(() -> {
              shooter.setShooterSpeed(0.7);
            })))
        .whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                  intake.setIntakeMotorSpeed(1, 0);
                }),
                new InstantCommand(() -> {
                  shooter.setShooterSpeed(0);
                })));

    // Lower intake down (B button)
    new JoystickButton(xbox, 2).whenPressed(() -> intake.setIntakeMotorSpeed(0,
        Constants.intakeSpeed))
        .whenReleased(() -> intake.setIntakeMotorSpeed(0, 0));

    // Button for disabling automatic operation
    new JoystickButton(xbox, 7).whenPressed(() -> {
      manualModeOp = true;
    }).whenReleased(() -> {
      manualModeOp = false;
    });
  }

  /** Returns whether or not the robot is driving at full speed. */
  public static boolean getBoostMode() {
    return boostMode;
  }

  /** Returns whether or not the robot is driving in slow mode. */
  public static boolean getSlowMode() {
    return slowMode;
  }

  /** Returns whether or not autonomous/smart systems are disabled. */
  public static boolean getManualMode() {
    return manualMode;
  }

  /** Returns whether or not automatic operation is distabled. */
  public static boolean getManualModeOp() {
    return manualModeOp;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory) {
    return null;
  }

  public Command getTelopCommand() {
    // Toggles dual joystick, should be replaced with an actual check in the future
    return new SingleJoystickDrive(m_driveTrain, stick1);
  }

}
