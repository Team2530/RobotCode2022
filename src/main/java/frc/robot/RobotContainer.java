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
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private final Hood m_hood = new Hood();
  private final Revolver m_revolver = new Revolver();
  private final Intake m_intake = new Intake();

  // -------------------- Joysticks and Buttons -------------------- \\
  // Joysticks
  final Joystick stick1 = new Joystick(Constants.stickport1); // Creates a joystick on port 1
  final Joystick stick2 = new Joystick(Constants.stickport2); // Creates a joystick on port 2

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
  final XboxController xbox = new XboxController(Constants.xboxport);

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
    new JoystickButton(stick1, 12).whenPressed(() -> m_hood.flywheelRotateSpeed(1))
        .whenReleased(() -> m_hood.flywheelRotateSpeed(0));
    // new JoystickButton(stick1, 3)
    // .whenPressed(() -> m_hood.flywheelRotateSpeed(0.9))
    // .whenReleased(() -> m_hood.setHoodPosition(0));
    // new JoystickButton(stick1, 4)
    // .whenPressed(() -> m_hood.flywheelRotateSpeed(0.9))
    // .whenReleased(() -> m_hood.setHoodPosition(0));

    // Rotates the revolver 90 degrees
    Button5.whenPressed(new TurnRevolver(m_revolver));

    // Manually rotates the revolver in the positive direction
    new JoystickButton(stick1, 2).whenPressed(() -> m_revolver.setRevolverSpeed(0.25))
        .whenReleased(() -> m_revolver.setRevolverSpeed(0));

    // Manually rotates the revolver in the negative direction
    new JoystickButton(stick1, 3).whenPressed(() -> m_revolver.setRevolverSpeed(-0.25))
        .whenReleased(() -> m_revolver.setRevolverSpeed(0));

    // Manually moves hood to specific angles
    new JoystickButton(stick1, 9).whileHeld(() -> m_hood.setHood(-1));
    new JoystickButton(stick1, 9).whenReleased(() -> m_hood.setHood(0));
    new JoystickButton(stick1, 10).whileHeld(() -> m_hood.setHood(1));
    new JoystickButton(stick1, 10).whenReleased(() -> m_hood.setHood(0));
    new JoystickButton(stick1, 7).whileHeld(() -> m_hood.setTurretPower(1));
    new JoystickButton(stick1, 7).whenReleased(() -> m_hood.setTurretPower(0));
    new JoystickButton(stick1, 8).whileHeld(() -> m_hood.setTurretPower(-1));
    new JoystickButton(stick1, 8).whenReleased(() -> m_hood.setTurretPower(0));

    new JoystickButton(stick1, 4).whenPressed(() -> m_hood.toggleAim());

    // Toggles the LimeLight camera mode (aiming to drive cam)
    new JoystickButton(stick1, 5).whenPressed(() -> m_hood.toggleCamMode());
    // Toggles the LimeLight LEDs (useful for not blinding people)
    new JoystickButton(stick1, 6).whenPressed(() -> m_hood.toggleLight());

    // Automatically shoots balls
    // new JoystickButton(xbox, 1).whenPressed(() -> new AutoShoot(m_revolver,
    // m_hood));

    // Intake control
    new JoystickButton(stick1, 13).whenPressed(() -> m_intake.setIntakeSpeed(0.5))
        .whenReleased(() -> m_intake.setIntakeSpeed(0));

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
    return new ParallelCommandGroup(new ManualAimHood(stick1, m_hood, m_revolver),
        new SingleJoystickDrive(m_driveTrain, stick1));
  }

}
