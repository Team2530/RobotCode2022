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
  // private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final USBCamera usbCamera = new USBCamera();

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
    // Climber control
    new JoystickButton(stick1, 4).whenPressed(() -> m_climber.setClimberSpeed(0.5))
        .whenReleased(() -> m_climber.setClimberSpeed(0));

    // new JoystickButton(stick1, 1).whenPressed(() ->
    // m_driveTrain.driveStraight(0.5))
    // .whenReleased(() -> m_driveTrain.driveStraight(0));
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
