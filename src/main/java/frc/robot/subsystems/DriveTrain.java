/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import frc.robot.libraries.Gains;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around, such as directly setting motor power,
 * handling the control mode, and auto-turning.
 */
public class DriveTrain extends SubsystemBase {
  public static enum DriveDirection {
    Forwards,
    Backwards,
    Left,
    Right
  };

  // -------------------- Motors -------------------- \\
  // Left Motors
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.MOTOR_FL_DRIVE_PORT);
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.MOTOR_FR_DRIVE_PORT);
  WPI_TalonFX motorBL = new WPI_TalonFX(Constants.MOTOR_BL_DRIVE_PORT);
  WPI_TalonFX motorBR = new WPI_TalonFX(Constants.MOTOR_BR_DRIVE_PORT);
  Joystick stick = new Joystick(Constants.stickport1);
  AHRS ahrs;

  // ------------------------ PID gains ------------------------- \\

  // "Teenage resistance" for rotation
  // private final double hkP = 0.05, hkI = 0.0015, hkD = 0.00175;
  private final Gains rotPIDGains = new Gains(Constants.rotPIDGainsP, Constants.rotPIDGainsI, Constants.rotPIDGainsD);

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
  PIDController turnRatePID = ratePIDGains.getPID();
  PIDController strafePID = drivePIDGains.getPID();
  PIDController drivePID = strafePIDGains.getPID();
  PIDController resistStrafePID = resistStrafePIDGains.getPID();
  PIDController resistDrivePID = resistDrivePIDGains.getPID();

  // ------------------------ States ------------------------- \\
  private static double[] position = { 0, 0, 0 };
  /** The actual joystick input on each axis. */
  private static double[] joystickInput = { 0, 0, 0 };
  /** The current joystick interpolation on each axis. */
  private static double[] joystickLerp = { 0, 0, 0 };
  /** Last joystick input when button 3 is pressed */
  private static double[] lastJoystickInput = { 0, 0, 0 };

  private double lastExecuted = Timer.getFPGATimestamp();

  public MecanumDrive mecanumDrive;

  // public final SimpleMotorFeedforward m_feedforward = new
  // SimpleMotorFeedforward(Constants.kS, Constants.kV,
  // Constants.kA);

  // public final PIDController m_leftPIDController = new
  // PIDController(Constants.PIDleftDrive.kP,
  // Constants.PIDleftDrive.kI, Constants.PIDleftDrive.kD);
  // public final PIDController m_rightPIDController = new
  // PIDController(Constants.PIDrigthDrive.kP,
  // Constants.PIDrigthDrive.kI, Constants.PIDrigthDrive.kD);

  // private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(0.125);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);

  /**
   * Creates a new {@link DriveTrain}.
   */
  public DriveTrain(AHRS ahrs) {
    // motorFL.configFactoryDefault();
    // motorFR.configFactoryDefault();
    // motorBL.configFactoryDefault();
    // motorBR.configFactoryDefault();
    setCoast(NeutralMode.Brake);
    motorFL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorBL.setSelectedSensorPosition(0);
    motorBR.setSelectedSensorPosition(0);
    // motorFL.feed();
    // motorFR.feed();
    // motorBL.feed();
    // motorBR.feed();
    motorFL.setInverted(false);
    motorBL.setInverted(false);
    motorFR.setInverted(true);
    motorBR.setInverted(true);

    mecanumDrive = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    mecanumDrive.setSafetyEnabled(false);

    lastExecuted = Timer.getFPGATimestamp();
    this.ahrs = ahrs;
  }

  @Override
  public void periodic() {
    putNavXInfo();
  }

  public void setCoast(NeutralMode neutralSetting) {
    motorFL.setNeutralMode(neutralSetting);
    motorFR.setNeutralMode(neutralSetting);
    motorBL.setNeutralMode(neutralSetting);
    motorBR.setNeutralMode(neutralSetting);
  }

  /**
   * Call this from the owner when the robot gets enabled, or you want to manually
   * re-center the stabilization
   */
  public void reset() {
    ahrs.zeroYaw();
    ahrs.resetDisplacement();
  }

  /**
   * Initializes a drive mode where only one joystick controls the drive motors.
   * 
   * @param x             The joystick's forward/backward tilt. Any value from
   *                      -1.0 to 1.0.
   * @param y             The joystick's sideways tilt. Any value from -1.0 to
   *                      1.0.
   * @param z             The joystick's vertical "twist". Any value from -1.0 to
   *                      1.0.
   */
  public void singleJoystickDrive(double x, double y, double z) {
    if (stick.getRawButton(Constants.velocityRetentionButton) == true) {
      x = lastJoystickInput[0];
      y = lastJoystickInput[1];
      z = 0;
    }
    if (stick.getRawButton(Constants.driveStraightButton) == true) {
      z = 0;
    }
    x = Deadzone.deadZone(x, Constants.deadzone);
    y = Deadzone.deadZone(y, Constants.deadzone);
    z = Deadzone.deadZone(z, Constants.deadzone);
    joystickInput[0] = x;
    joystickInput[1] = y;
    joystickInput[2] = z;
    // Do for each joystick axis
    for (int axis = 0; axis < 3; ++axis) {
      // Is the magnitude of the actual joystick input greater than the magnitude of
      // the current joystick interpolation?
      boolean isIncreasing = Math.abs(joystickInput[axis]) > Math.abs(joystickLerp[axis]);
      // Is the difference between the actual and interpolated joystick input greater
      // than the acceptable margin?
      boolean isOutsideMargin = Math.abs(joystickLerp[axis] - joystickInput[axis]) > Constants.DRIVE_RAMP_INTERVAL;
      if (!RobotContainer.getManualMode() && isIncreasing && isOutsideMargin) {
        // If we're not there yet
        joystickLerp[axis] = (joystickLerp[axis]
            + Constants.DRIVE_RAMP_INTERVAL * Math.signum(joystickInput[axis] - joystickLerp[axis]));
      } else {
        // If our patience has paid off
        joystickLerp[axis] = joystickInput[axis];
      }
      if (!stick.getRawButton(Constants.velocityRetentionButton)) {
        lastJoystickInput[0] = joystickInput[1];
        lastJoystickInput[1] = -joystickInput[0];
      }
    }

    actuallyDrive(joystickLerp[1], -joystickLerp[0], joystickLerp[2]);
  }

  public void actuallyDrive(double x, double y, double z) {
    // TODO : Test deadzone
    // mecanumDrive.driveCartesian(y, -x, -z);

    // navX coordinates:
    // +X = drive forward, -X = drive backward
    // +Y = strafe left, -Y = strafe right
    // TEST: X and Y axis velocity PIDs and independent deadzones

    // PID control for robot forward/backward/strafing control
    // TODO: Double check strafe speed calculation

    double deltaTime = Timer.getFPGATimestamp() - lastExecuted;
    lastExecuted = Timer.getFPGATimestamp();

    double yPIDCalc, xPIDCalc, yTarget, xTarget;
    if (Math.abs(y) < 0.1 && Math.abs(x) < 0.1) {
      // If we're not intentionally strafing or driving forwards/backwards, engage
      // teenage resistance (positional lock)
      yTarget = ahrs.getDisplacementY() + y * Constants.maxMetersPerSecondStrafe * deltaTime;
      yPIDCalc = Deadzone.cutOff(-resistStrafePID.calculate(ahrs.getDisplacementY() / 360.0, yTarget / 360) * 0.25,
          0.01);
      xTarget = ahrs.getDisplacementX() + x * Constants.maxMetersPerSecondForwards * deltaTime;
      xPIDCalc = Deadzone.cutOff(-resistStrafePID.calculate(ahrs.getDisplacementX() / 360.0, xTarget / 360) * 0.25,
          0.01);
    } else {
      // If we *are* intentionally strafing, use regular velocity control
      ahrs.resetDisplacement();
      yPIDCalc = strafePID.calculate(ahrs.getVelocityY(),
          Deadzone.deadZone(y, 0.1) * Constants.maxMetersPerSecondStrafe * deltaTime);
      xPIDCalc = drivePID.calculate(ahrs.getVelocityX(),
          Deadzone.deadZone(x, 0.1) * Constants.maxMetersPerSecondForwards * deltaTime);
    }

    // PID control for robot rotation
    double zPIDCalc, zTarget;
    if (Math.abs(z) < 0.1) {
      // If we're not intentionally turning, engage teenage resistance (directional
      // lock)
      zPIDCalc = Deadzone.cutOff(-rotPID.calculate(ahrs.getAngle() / 360.0, position[2] / 360), 0.01);
    } else {
      // If we *are* intentionally turning, use regular velocity control
      // zPIDCalc = turnRatePID.calculate(ahrs.getRate(),
      //     Deadzone.deadZone(-z, 0.1) * Constants.maxDegreesPerSecondRotate * deltaTime);
      position[2] = ahrs.getAngle();
      zPIDCalc = z;
    }

    mecanumDrive.driveCartesian(yPIDCalc, xPIDCalc, zPIDCalc);
  }

  public void stop() {
    mecanumDrive.stopMotor();
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
