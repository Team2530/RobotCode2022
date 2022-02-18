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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  // -------------------- Joysticks and Buttons -------------------- \\
  // Joysticks
  final Joystick stick1 = new Joystick(0); // Creates a joystick on port 1
  final Joystick stick2 = new Joystick(1); // Creates a joystick on port 2

  // Joystick buttons
  // private final JoystickButton Button1 = new JoystickButton(stick1, 1); //
  // Creates a new button for button 1 on stick1
  // private final JoystickButton Button2 = new JoystickButton(stick1, 2);
  // private final JoystickButton Button3 = new JoystickButton(stick1, 3);
  // private final JoystickButton Button4 = new JoystickButton(stick1, 4);
  private final JoystickButton Button5 = new JoystickButton(stick1, 5);
  // private final JoystickButton Button6 = new JoystickButton(stick1, 6);
  // private final JoystickButton Button7 = new JoystickButton(stick1, 7);
  // private final JoystickButton Button9 = new JoystickButton(stick1, 9);
  // private final JoystickButton Button10 = new JoystickButton(stick1, 10);
  // private final JoystickButton Button11 = new JoystickButton(stick1, 11);
  // private final JoystickButton Button12 = new JoystickButton(stick1, 12);
  // Xbox Controller
  final XboxController xbox = new XboxController(0);

  // Xbox buttons
  // private final JoystickButton XboxButton1 = new JoystickButton(xbox, 1);

  // -------------------- Autonomous Commands -------------------- \\

  String trajectoryJSON = "PathWeaver/DriveForwardFarBlue.wpilib.json";
  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 60);

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  // private final TrajectoryTest m_autoCommand = new TrajectoryTest(m_driveTrain,
  // new Traj);

  // private final DelayTest delayCommand = new DelayTest(1, m_autoCommand);

  // -------------------- Telop Commands -------------------- \\
  // private final XboxJoystickElevator elevatorCommand = new
  // XboxJoystickElevator(elevatorSub, xbox);
  // private final SmallJoystickElevator elevatorCommand = new
  // SmallJoystickElevator(elevatorSub, stick1);
  // private final EncoderTest m_telopCommand = new EncoderTest(m_driveTrain);
  // private final TurnRevolver turnRevolver = new
  // TurnRevolver(m_revolver,stick1);

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
