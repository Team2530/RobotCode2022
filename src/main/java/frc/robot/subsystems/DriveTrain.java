/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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
  XboxController xbox = new XboxController(Constants.xboxport);
  AHRS ahrs;
  Timer timer = new Timer();

  // --------------------Field2d Stuff------------------------\\
  Field2d m_field = new Field2d();
  Pose2d m_pose = new Pose2d();
  Rotation2d m_rotation = new Rotation2d();
  Rotation2d rotation;
  double fieldXPos = 8.0;
  double fieldYPos = 4.0;
  double fieldRotation = 0.0;
  /**
   * Speed for Field2d (X, Y, Rotation)
   */
  double fieldSpeed[] = { 0.0, 0.0, 0.0 };

  /** The actual joystick input on each axis. */
  public static double[] joystickInput = { 0, 0, 0 };
  /** The current joystick interpolation on each axis. */
  private static double[] joystickLerp = { 0, 0, 0 };
  /** Last joystick input when button 3 is pressed */
  private static double[] lastJoystickInput = { 0, 0, 0 };

  // NOTE: Yaw is in degrees, need small pid constants
  // private final double kP = 0.05, kI = 0.0015, kD = 0.00175;
  private final double kP = 18.7, kI = 1.7, kD = 1.4;
  PIDController rot_pid = new PIDController(kP, kI, kD);

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

    this.ahrs = ahrs;
  }

  @Override
  public void periodic() {
    // putNavXInfo();
    getBatteryRuntime();
    field2d();
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
    if (stick.getRawButton(Constants.velocityRetentionButton) == true) {
      x = lastJoystickInput[0];
      y = lastJoystickInput[1];
      z = 0;
    }
    if (stick.getRawButton(Constants.driveStraightButton) == true) {
      z = 0;
    }
    if (stick.getPOV() != -1) {
      double[] driveStraight = actuallyDriveStraighter(x, y);
      x = driveStraight[0];
      y = driveStraight[1];
      z = 0;
    }
    if (stick.getPOV() == -1) {
      mecanumDrive.driveCartesian(
          -y, -x, -z, ahrs.getYaw()); // -rot_pid.calculate(ahrs.getAngle() / 360.0, yawTarget / 360) * 0.25);
    } else {
      mecanumDrive.driveCartesian(-y, -x, -z);
    }
      SmartDashboard.putNumber("twist", Deadzone.deadZone(-z,
          Constants.deadzoneZ));
    }
  
  public void stop() {
    mecanumDrive.stopMotor();
  }

  public double[] actuallyDriveStraighter(double x, double y) {
    double[] result = {x, y};
    int POV = stick.getPOV();
    if (POV == 0) {
      result[0] = 0;
      result[1] = 0.2;
    } else if (POV == 45) {
      result[0] = 0.1;
      result[1] = 0.1;
    } else if (POV == 90) {
      result[0] = 0.2;
      result[1] = 0;
    } else if (POV == 135) {
      result[0] = 0.1;
      result[1] = -0.1;
    } else if (POV == 180) {
      result[0] = 0;
      result[1] = -0.2;
    } else if (POV == 225) {
      result[0] = -0.1;
      result[1] = -0.1;
    } else if (POV == 270) {
      result[0] = -0.2;
      result[1] = 0;
    } else if (POV == 315) {
      result[0] = -0.1;
      result[1] = 0.1;
    }
    return result;
  }

  // public void putNavXInfo() {
  // SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
  // SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
  // SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
  // SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
  // SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
  // SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
  // SmartDashboard.putNumber("Accumulated yaw ", ahrs.getAngle());
  // SmartDashboard.putNumber("Rotational velocity (raw)", ahrs.getRawGyroZ());
  // }

  public void getBatteryRuntime() {
    double a = Math.abs(motorBL.getMotorOutputVoltage());
    double b = Math.abs(motorBR.getMotorOutputVoltage());
    double q = Math.abs(motorFL.getMotorOutputVoltage());
    double e = Math.abs(motorFR.getMotorOutputVoltage());

    if ((a > .1) || (b > .1) || (q > .1) || (e > .1) || xbox.getRawButton(3) || xbox.getRawButton(1)
        || xbox.getRawButton(2)) {
      timer.start();
      SmartDashboard.putNumber("Battery Runtime", timer.get());
    } else {
      timer.stop();
    }
  }

  public void field2d() {
    m_field.setRobotPose(fieldXPos, fieldYPos, m_rotation);
    SmartDashboard.putData(m_field);
    // NavX Rotated 90
    fieldXPos = fieldXPos + ahrs.getVelocityY();
    fieldYPos = fieldYPos + ahrs.getVelocityX();
    fieldRotation = fieldRotation + ahrs.getVelocityZ();
     // Calculations for Movement, Physics, etc...
    // field2dSimuationMode();
    
  }

/**
 * Makes sure the robot dosen't go off screen
 */
  public void field2dBounds() {
    if (fieldXPos > 16) {
      fieldXPos = 16;
    }
    if (fieldXPos < 0) {
      fieldXPos = 0;
    }
    if (fieldYPos < 0) {
      fieldYPos = 0;
    }
    if (fieldYPos > 8) {
      fieldYPos = 8;
    }
  }
  /** To use when using field2d on simuation mode */
  public void field2dSimuationMode(){
    if (Math.abs(joystickInput[1]) >= .1) {
      fieldSpeed[0] = (joystickLerp[1] / 8);
      } else {
      fieldSpeed[0] = fieldSpeed[0] * 0.92;
    }
    fieldXPos = fieldXPos + fieldSpeed[0];
    if (Math.abs(joystickInput[0]) >= .1) {
      fieldSpeed[1] = (joystickLerp[0] / 8);
    } else {
      fieldSpeed[1] = fieldSpeed[1] * 0.92;
    }
    fieldYPos = fieldYPos - fieldSpeed[1];
    if (Math.abs(joystickInput[2]) >= .5) {
      fieldSpeed[2] = fieldSpeed[2] + (joystickLerp[2] / 3);
    } else {
      fieldSpeed[2] = fieldSpeed[2] * 0.8;
    }
    fieldRotation = fieldRotation - fieldSpeed[2];
    m_rotation = Rotation2d.fromDegrees(fieldRotation);
    field2dBounds();
  }
}