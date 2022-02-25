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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  // The robot's subsystems and commands are defined here...

  // -------------------- Subsystems -------------------- \\

  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Climber m_climber = new Climber();
  private final USBCamera usbCamera = new USBCamera();
  private final Intake intake = new Intake();
  private final Chambers ballDetection = new Chambers(3);
  private final LimeLight m_limeLight = new LimeLight();

  // -------------------- Joysticks and Buttons -------------------- \\
  // Joysticks
  final Joystick stick1 = new Joystick(Constants.stickport1); // Creates a joystick on port 1
  final Joystick stick2 = new Joystick(Constants.stickport2); // Creates a joystick on port 2

  // Xbox Controller
  final XboxController xbox = new XboxController(Constants.xboxport);

  // -------------------- Autonomous Commands -------------------- \\
  // insert autonomous commands here

  // -------------------- Telop Commands -------------------- \\
  // insert teleop commands here

  // -------------------- Global Toggles -------------------- \\
  /** Whether or not autonomous/smart systems are disabled. */
  private static boolean manualMode = false;

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
    // Button for disabling autonomous/smart functions
    new JoystickButton(stick1, 2).whenPressed(() -> {
      manualMode = true;
    }).whenReleased(() -> {
      manualMode = false;
    });

    // Climber control
    new JoystickButton(xbox, 6).whenPressed(() -> m_climber.setClimberSpeed(1.0))
        .whenReleased(() -> m_climber.setClimberSpeed(0));
    new JoystickButton(xbox, 5).whenPressed(() -> m_climber.setClimberSpeed(0.1))
        .whenReleased(() -> m_climber.setClimberSpeed(0));
        
    // Toggles the LimeLight camera mode (aiming to drive cam)
    new JoystickButton(stick1, 5).whenPressed(() -> m_limeLight.toggleCamMode());
    // Toggles the LimeLight LEDs (useful for blinding people)
    new JoystickButton(stick1, 3).whenPressed(() -> m_limeLight.toggleLight());

    // Lower intake up
    new JoystickButton(xbox, 1).whenPressed(() -> intake.setIntakeMotorSpeed(0, -0.75))
        .whenReleased(() -> intake.setIntakeMotorSpeed(0, 0));

    // Upper intake up
    new JoystickButton(xbox, 3).whenPressed(() -> intake.setIntakeMotorSpeed(1, -0.75))
        .whenReleased(() -> intake.setIntakeMotorSpeed(1, 0));

    // Lower intake down
    new JoystickButton(xbox, 2).whenPressed(() -> intake.setIntakeMotorSpeed(0, 0.75))
        .whenReleased(() -> intake.setIntakeMotorSpeed(0, 0));

    // Zero navX rotation
    new JoystickButton(stick1, 8).whenPressed(() -> m_driveTrain.reset());

    // new JoystickButton(stick1, 1).whenPressed(() ->
    // m_driveTrain.driveStraight(0.5))
    // .whenReleased(() -> m_driveTrain.driveStraight(0));
  }

  /** Returns whether or not autonomous/smart systems are disabled. */
  public static boolean getManualMode() {
    return manualMode;
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
