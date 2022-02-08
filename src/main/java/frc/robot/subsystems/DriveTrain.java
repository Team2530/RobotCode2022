/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  AHRS ahrs = new AHRS();

  // ------------------------ PID gains ------------------------- \\

  // "Teenage resistance"
  // private final double hkP = 0.05, hkI = 0.0015, hkD = 0.00175;
  private final Gains rotPIDGains = new Gains(18.7, 1.7, 1.4);

  // Turn rate control (Z angular velocity control)
  private final Gains ratePIDGains = new Gains(1.0, 0.0, 0.0); // TODO: Tune

  // Left and right rate control
  private final Gains strafePIDGains = new Gains(1.0, 0.0, 0.0); // TODO: Tune

  // Forward and back rate control
  private final Gains drivePIDGains = new Gains(1.0, 0.0, 0.0); // TODO: Tune

  PIDController rotPID = rotPIDGains.getPID();
  PIDController turnrate_pid = ratePIDGains.getPID();

  PIDController strafePID = drivePIDGains.getPID();
  PIDController drivePID = strafePIDGains.getPID();

  private final double maxMetersPerSecondForwards = 1.0;
  private final double maxMetersPerSecondStrafe = maxMetersPerSecondForwards / Math.sqrt(2); // TODO: Actually test

  public double yawTarget = 0.0;
  public final double yawRate = 310.0;

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
  public DriveTrain() {
    // motorFL.configFactoryDefault();
    // motorFR.configFactoryDefault();
    // motorBL.configFactoryDefault();
    // motorBR.configFactoryDefault();
    setCoast(NeutralMode.Brake);
    motorFR.setInverted(true);
    motorBR.setInverted(true);
    motorFL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorBL.setSelectedSensorPosition(0);
    motorBR.setSelectedSensorPosition(0);
    // motorFL.feed();
    // motorFR.feed();
    // motorBL.feed();
    // motorBR.feed();
    motorFR.setInverted(true);
    motorBR.setInverted(true);

    mecanumDrive = new MecanumDrive(motorFL, motorBL, motorFR, motorBR);
    mecanumDrive.setSafetyEnabled(true);
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
   * @param headingAdjust when true, yaw control is set to heading hold and the z
   *                      (steering) axis instead adjusts the yaw target
   */
  public void singleJoystickDrive(double x, double y, double z, double deltaTime, boolean headingAdjust) {
    // mecanumDrive.driveCartesian(y, -x, -z);
    // TODO: Deadzones for pid output

    /*
     * 
     * TODO: While driving, PID for turn rate (gyro angular velocity Z axis,
     * `getRate()`)
     * TODO: While not turning, lock heading and use the "teenage resistance" PID to
     * drive
     * straight.
     * 
     * TEST: X and Y axis velocity PIDs and independent deadzones
     * TODO: Position-locking PID when not moving? It could even have a really high
     * I setting to resist being shoved.
     * 
     */

    // NOTE: Disabled because rate/heading switchover not implemented
    // if (Math.abs(z) >= 0.1)
    // yawTarget = ahrs.getAngle();

    if (headingAdjust)
      yawTarget += z * yawRate * deltaTime;

    // Rate control driving for x/y, heading select steering currently, needs work
    // and testing
    mecanumDrive.driveCartesian(
        // TODO: Double check the accelerometer orientation on the robot
        // TODO: Double check strafe speed calculation
        strafePID.calculate(ahrs.getVelocityX(), Deadzone.deadZone(y, 0.1) * maxMetersPerSecondStrafe),
        drivePID.calculate(ahrs.getVelocityY(), Deadzone.deadZone(-x, 0.1) * maxMetersPerSecondForwards),
        // TODO: Rate control when turning, otherwise lock heading for stability
        Deadzone.cutOff(-rotPID.calculate(ahrs.getAngle() / 360.0, yawTarget / 360) * 0.25, 0.01));
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

  /**
   * Rotates 180, Why not?
   */
  public void deathBlossom(){
    yawTarget += 180;
  }



}
