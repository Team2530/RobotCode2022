/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.libraries.Gains;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around.
 */
public class DriveTrain extends SubsystemBase {
  public static enum Cockpit {
    FRONT,
    LEFT,
    RIGHT
  }
  private static Cockpit cockpitMode; 

  /** The actual joystick input on each axis. */
  public static double[] joystickInput = { 0, 0, 0 };
  /** The current joystick interpolation on each axis. */
  public static double[] joystickLerp = { 0, 0, 0 };
  /** Last joystick input when button 3 is pressed */
  private static double[] lastJoystickInput = { 0, 0, 0 };

  private static double yawTarget = 0.0;

  private double lastExecuted = Timer.getFPGATimestamp();

  public MecanumDrive mecanumDrive;

  // -------------------- Motors -------------------- \\
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.MOTOR_FL_DRIVE_PORT);
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.MOTOR_FR_DRIVE_PORT);
  WPI_TalonFX motorBL = new WPI_TalonFX(Constants.MOTOR_BL_DRIVE_PORT);
  WPI_TalonFX motorBR = new WPI_TalonFX(Constants.MOTOR_BR_DRIVE_PORT);
  Joystick stick;
  XboxController xbox;
  AHRS ahrs;

  // ------------------------ PID gains ------------------------- \\

  // Slider Instances
  NetworkTableEntry rotPidP = Shuffleboard.getTab("PID Constants").add("Rot P", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  NetworkTableEntry rotPidI = Shuffleboard.getTab("PID Constants").add("Rot I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  NetworkTableEntry rotPidD = Shuffleboard.getTab("PID Constants").add("Rot D", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)).getEntry();

  // Slider Values
  static double ROT_PID_P = 0;
  static double ROT_PID_I = 0;
  static double ROT_PID_D = 0;

  // "Teenage resistance" for rotation
  // private final double hkP = 0.05, hkI = 0.0015, hkD = 0.00175;
  private final Gains rotPIDGains = new Gains(
      Constants.rotPIDGainsP == 0 ? ROT_PID_P : Constants.rotPIDGainsP,
      Constants.rotPIDGainsI == 0 ? ROT_PID_I : Constants.rotPIDGainsI,
      Constants.rotPIDGainsD == 0 ? ROT_PID_D : Constants.rotPIDGainsD);

  // Rotation velocity control (Z angular velocity control)
  private final Gains ratePIDGains = new Gains(Constants.ratePIDGainsP, Constants.ratePIDGainsI,
      Constants.ratePIDGainsD);

  // "Teenage resistance" for strafing
  private final Gains resistStrafePIDGains = new Gains(Constants.resistStrafePIDGainsP, Constants.resistStrafePIDGainsI,
      Constants.resistStrafePIDGainsD);

  // Left and right velocity control
  private final Gains strafePIDGains = new Gains(Constants.strafePIDGainsP, Constants.strafePIDGainsI,
      Constants.strafePIDGainsD);

  // "Teenage resistance" for forward/backward
  private final Gains resistDrivePIDGains = new Gains(Constants.resistDrivePIDGainsP, Constants.resistDrivePIDGainsI,
      Constants.resistDrivePIDGainsD);

  // Forward and backward velocity control
  private final Gains drivePIDGains = new Gains(Constants.drivePIDGainsP, Constants.drivePIDGainsI,
      Constants.drivePIDGainsD);

  PIDController rotPID = rotPIDGains.getPID();
  PIDController resistStrafePID = resistStrafePIDGains.getPID();
  PIDController resistDrivePID = resistDrivePIDGains.getPID();
  PIDController turnRatePID = ratePIDGains.getPID();
  PIDController strafePID = drivePIDGains.getPID();
  PIDController drivePID = strafePIDGains.getPID();

  /**
   * Creates a new {@link DriveTrain}.
   */
  public DriveTrain(AHRS ahrs, Joystick stick, XboxController xbox) {
    this.ahrs = ahrs;
    this.stick = stick;
    this.xbox = xbox;
    lastExecuted = Timer.getFPGATimestamp();

    mecanumDrive = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    mecanumDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // putNavXInfo();
    SmartDashboard.putNumber("rotPIDGraph", rotPID.getPositionError());
    ROT_PID_P = rotPidP.getDouble(ROT_PID_P);
    ROT_PID_I = rotPidI.getDouble(ROT_PID_I);
    ROT_PID_D = rotPidD.getDouble(ROT_PID_D);
    rotPID.setPID(
        Constants.rotPIDGainsP == 0 ? ROT_PID_P : Constants.rotPIDGainsP,
        Constants.rotPIDGainsI == 0 ? ROT_PID_I : Constants.rotPIDGainsI,
        Constants.rotPIDGainsD == 0 ? ROT_PID_D : Constants.rotPIDGainsD);
  }

  /**
   * Call this from the owner when the robot gets enabled, or you want to manually
   * re-center the stabilization
   */
  public void reset() {
    // ahrs.enableBoardlevelYawReset(true);
    ahrs.reset();
    ahrs.zeroYaw();
    ahrs.resetDisplacement();
    yawTarget = 0.0;
    cockpitMode = Cockpit.FRONT;

    setCoast(NeutralMode.Brake);
    motorFL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorBL.setSelectedSensorPosition(0);
    motorBR.setSelectedSensorPosition(0);
    motorFL.setInverted(false);
    motorBL.setInverted(false);
    motorFR.setInverted(true);
    motorBR.setInverted(true);
  }

  public void setCoast(NeutralMode neutralSetting) {
    motorFL.setNeutralMode(neutralSetting);
    motorFR.setNeutralMode(neutralSetting);
    motorBL.setNeutralMode(neutralSetting);
    motorBR.setNeutralMode(neutralSetting);
  }

  /**
   * Initializes a drive mode where only one joystick controls the drive motors.
   * 
   * @param x The joystick's forward/backward tilt. Any value from
   *          -1.0 to 1.0.
   * @param y The joystick's sideways tilt. Any value from -1.0 to
   *          1.0.
   * @param z The joystick's vertical "twist". Any value from -1.0 to
   *          1.0.
   */
  public void singleJoystickDrive(double x, double y, double z) {
    if (stick.getRawButton(Constants.velocityRetentionButton)) {
      x = lastJoystickInput[0];
      y = lastJoystickInput[1];
      z = 0;
    } else {
      lastJoystickInput[0] = joystickInput[1];
      lastJoystickInput[1] = -joystickInput[0];
      if (stick.getRawButton(Constants.driveStraightButton)) {
        z = 0;
      } else if (stick.getPOV() != -1) {
        double[] driveStraight = actuallyDriveStraighter(x, y);
        y = driveStraight[0];
        x = -driveStraight[1];
        z = 0;
      }
    }

    // Keep track of unmodified joystick input
    joystickInput[0] = x;
    joystickInput[1] = y;
    joystickInput[2] = z;

    // Drive gradient (ramping)
    for (int axis = 0; axis < 3; ++axis) {
      boolean isIncreasing = Math.abs(joystickInput[axis]) > Math.abs(joystickLerp[axis]);
      boolean isOutsideMargin = Math.abs(joystickLerp[axis] - joystickInput[axis]) > Constants.DRIVE_RAMP_INTERVAL;
      if (!RobotContainer.getManualMode() && isIncreasing && isOutsideMargin) {
        // If we're not there yet
        joystickLerp[axis] = (joystickLerp[axis]
            + Constants.DRIVE_RAMP_INTERVAL * Math.signum(joystickInput[axis] - joystickLerp[axis]));
      } else {
        // If our patience has paid off
        joystickLerp[axis] = joystickInput[axis];
      }
    }

    x = joystickLerp[1];
    y = -joystickLerp[0];
    z = joystickLerp[2];

    // navX coordinates:
    // +X = drive forward, -X = drive backward
    // +Y = strafe left, -Y = strafe right
    // TEST: X and Y axis velocity PIDs and independent deadzones

    // PID control for robot forward/backward/strafing control
    // TODO: Double check strafe speed calculation

    double deltaTime = Timer.getFPGATimestamp() - lastExecuted;
    lastExecuted = Timer.getFPGATimestamp();

    double yPIDCalc, xPIDCalc;
    if (Math.abs(y) == 0 && Math.abs(x) == 0) {
      // If we're not intentionally strafing or driving forwards/backwards, engage
      // teenage resistance (positional lock)
      yPIDCalc = resistStrafePID.calculate(ahrs.getDisplacementY() / (Constants.maxMetersPerSecondStrafe * deltaTime),
          0);
      xPIDCalc = resistStrafePID
          .calculate(ahrs.getDisplacementX() / (Constants.maxMetersPerSecondForwards * deltaTime), 0);
    } else {
      // If we *are* intentionally strafing or driving, keep track of the current
      // position
      ahrs.resetDisplacement();
      yPIDCalc = y;
      xPIDCalc = x;
      // TODO: Transition back to velocity PIDs
      // yPIDCalc = strafePID.calculate(ahrs.getVelocityY(),
      // Deadzone.deadZone(y, 0.1) * Constants.maxMetersPerSecondStrafe * deltaTime);
      // xPIDCalc = drivePID.calculate(ahrs.getVelocityX(),
      // Deadzone.deadZone(x, 0.1) * Constants.maxMetersPerSecondForwards *
      // deltaTime);
    }

    // PID control for robot rotation
    double zPIDCalc;
    if (Math.abs(z) == 0) {
      // If we're not intentionally turning, engage teenage resistance (directional
      // lock)
      zPIDCalc = Math.min(rotPID.calculate(ahrs.getAngle(), yawTarget), 0.5);
    } else {
      // If we *are* intentionally turning, keep track of the current angle
      yawTarget = ahrs.getAngle();
      SmartDashboard.putNumber("target angle", yawTarget);
      SmartDashboard.putNumber("actual angle", ahrs.getAngle());
      zPIDCalc = z;
      // TODO: Transition back to velocity PIDs
      // zPIDCalc = turnRatePID.calculate(ahrs.getRate(),
      // Deadzone.deadZone(-z, 0.1) * Constants.maxDegreesPerSecondRotate *
      // deltaTime);
    }
    SmartDashboard.putString("Cockpit mode", cockpitMode == Cockpit.FRONT ? "Front" : cockpitMode == Cockpit.LEFT ? "Left" : "Right");
    if (cockpitMode == Cockpit.FRONT) {
      driveFieldOriented(xPIDCalc, yPIDCalc, zPIDCalc);
    } else if (cockpitMode == Cockpit.LEFT) {
      driveOrientedToAngle(xPIDCalc, yPIDCalc, zPIDCalc, -90.0);
    } else if (cockpitMode == Cockpit.RIGHT) {
      driveOrientedToAngle(xPIDCalc, yPIDCalc, zPIDCalc, 90.0);
    }
  }

  /**
   * Drives the robot in a direction relative to itself.
   * 
   * @param x Positive is strafing right, negative is strafing left
   * @param y Positive is driving forward, negative is driving backward
   * @param z I have no idea
   */
  public void driveRobotOriented(double x, double y, double z) {
    driveOrientedToAngle(x, y, z, 0.0);
  }

  public void driveFieldOriented(double x, double y, double z) {
    driveOrientedToAngle(x, y, z, ahrs.getYaw());
  }

  /**
   * Drives the robot in a direction relative to where the navX was last
   * calibrated.
   * 
   * @param x Positive is strafing right, negative is strafing left
   * @param y Positive is driving forward, negative is driving backward
   * @param z I have no idea
   */
  public void driveOrientedToAngle(double x, double y, double z, double angle) {
    double driveX = Deadzone.cutOff(-y, Constants.cutOffMotorSpeed) * Constants.maxDriveSpeed;
    double driveY = Deadzone.cutOff(-x, Constants.cutOffMotorSpeed) * Constants.maxDriveSpeed;
    double driveZ = Deadzone.cutOff(-z, Constants.cutOffMotorSpeed) * Constants.maxDriveSpeed;
    mecanumDrive.driveCartesian(driveX, driveY, driveZ, angle);
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

  public double[] actuallyDriveStraighter(double x, double y) {
    double[] result = { x, y };
    int POV = stick.getPOV();
    if (POV == 0) {
      result[0] = 0;
      result[1] = 0.2;
    } else if (POV == 90) {
      result[0] = 0.4;
      result[1] = 0;
    } else if (POV == 180) {
      result[0] = 0;
      result[1] = -0.2;
    } else if (POV == 270) {
      result[0] = -0.4;
      result[1] = 0;
    }
    return result;
  }

  public void toggleIntakeCockpit() {
    if (cockpitMode == Cockpit.FRONT) {
      cockpitMode = Cockpit.LEFT;
    } else if (cockpitMode == Cockpit.LEFT) {
      cockpitMode = Cockpit.RIGHT;
    } else if (cockpitMode == Cockpit.RIGHT) {
      cockpitMode = Cockpit.FRONT;
    }
    USBCamera.changeCameraSource(cockpitMode);
  }

  /** Rotates 180, why not? */
  public void deathBlossom() {
    yawTarget += 180;
  }

  public void putNavXInfo() {
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
    SmartDashboard.putNumber("Accumulated yaw ", ahrs.getAngle());
    SmartDashboard.putNumber("Rotational velocity (raw)", ahrs.getRawGyroZ());
  }
}