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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.Cockpit;
import frc.robot.subsystems.FeedbackPanel.PanelMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

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

  // Inputs
  private final AHRS m_ahrs = new AHRS();
  private final USBCamera usbCamera = new USBCamera();

  // Outputs
  public final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, stick1, xbox);
  private final Climber m_climber = new Climber();
  private final Intake intake = new Intake();
  private final Chambers ballDetection = new Chambers(3);
  private final Indicators lights = new Indicators(3);
  private final PhotonVision vision = new PhotonVision(stick1, m_driveTrain, lights);
  private final FeedbackPanel m_feedbackPanel = new FeedbackPanel(3);
  private final Shooter shooter = new Shooter(xbox);

  // Diagnostics
  private final Battery m_battery = new Battery(m_ahrs, m_driveTrain, xbox);
  private final Field field = new Field(m_ahrs);

  // -------------------- Autonomous Commands -------------------- \\
  // insert autonomous commands here

  // -------------------- Telop Commands -------------------- \\
  // insert teleop commands here

  // -------------------- Global Toggles -------------------- \\
  /** Whether or not autonomous/smart systems are disabled. */
  private static boolean manualMode = false;
  private static boolean boostMode = false;
  private static boolean slowMode = false;
  public static boolean manualModeOp = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Switch Shuffleboard to the driver dashboard
    Shuffleboard.getTab("Config");
    Shuffleboard.selectTab("Config");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Button for driving at full speed
    new JoystickButton(stick1, 5).whenPressed(() -> {
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

    new JoystickButton(stick1, 9).whenPressed(() -> {
    }).whenReleased(() -> {
      vision.invertCams();
    });

    new JoystickButton(stick1, Constants.VISION_DRIVE_BUTTON).whenPressed(() -> {
      vision.drive();
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
              // Normal Shooter Operation
              // shooter.setShooterSpeed(0.7);
              // Change shooter with xbox triggers
              shooter.setShooterSpeed(Shooter.shooterSpeedWithTriggerChange);
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

    // Toggle driving in intake-oriented mode
    new JoystickButton(stick1, 7).whenPressed(() -> m_driveTrain.setIntakeCockpitMode(Cockpit.LEFT))
        .whenReleased(() -> {
          if (m_driveTrain.getIntakeCockpitMode() == Cockpit.LEFT)
            m_driveTrain.setIntakeCockpitMode(Cockpit.NONE);
        });
    new JoystickButton(stick1, 8).whenPressed(() -> m_driveTrain.setIntakeCockpitMode(Cockpit.RIGHT))
        .whenReleased(() -> {
          if (m_driveTrain.getIntakeCockpitMode() == Cockpit.RIGHT)
            m_driveTrain.setIntakeCockpitMode(Cockpit.NONE);
        });

    // Toggle driving in robot-oriented mode
    new JoystickButton(stick1, 11).whenPressed(() -> m_driveTrain.setIntakeCockpitMode(Cockpit.FRONT))
        .whenReleased(() -> {
          if (m_driveTrain.getIntakeCockpitMode() == Cockpit.FRONT)
            m_driveTrain.setIntakeCockpitMode(Cockpit.NONE);
        });

    // Death Blossom (rotate 180)
    new JoystickButton(stick1, 12).whenPressed(() -> m_driveTrain.deathBlossom(180));
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
   * Use this to pass the enable command to the main {@link Robot} class.
   * This command is run immediately when the robot is enabled (not simply turned
   * on), regardless of whether the robot is in teleop or autonomous.
   *
   * @return the command to run when the robot is enabled
   */
  public Command getEnableCommand() {
    return new InstantCommand(() -> m_driveTrain.reset(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Creates a new Autonomous Command for the robot
    System.out.println("Getting autonomous command");
    m_feedbackPanel.setDisplayMode(PanelMode.Boot);
    return new Autonomous(m_driveTrain, intake, shooter, m_ahrs, xbox, m_feedbackPanel);
  }

  public Command getTelopCommand() {
    vision.setPipelineFromAlliance();
    m_feedbackPanel.setDisplayMode(PanelMode.Status);
    Shuffleboard.selectTab("Driver Dashboard");
    return new SingleJoystickDrive(m_driveTrain, stick1);
  }

  public Command getTestCommand() {
    return new SingleJoystickDrive(m_driveTrain, stick1);
  }
}
