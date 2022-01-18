/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * This is Team 2530's DriveTrain class. It handles all things related to the
 * motors used to drive the robot around, such as directly setting motor power,
 * handling the control mode, and auto-turning.
 */
public class DriveTrain extends SubsystemBase {
  // -------------------- Motors -------------------- \\
  // Left Motors
  WPI_TalonFX motorFL = new WPI_TalonFX(Constants.MOTOR_FL_DRIVE_PORT);
  WPI_TalonFX motorFR = new WPI_TalonFX(Constants.MOTOR_FR_DRIVE_PORT);
  WPI_TalonFX motorBL = new WPI_TalonFX(Constants.MOTOR_BL_DRIVE_PORT);
  WPI_TalonFX motorBR = new WPI_TalonFX(Constants.MOTOR_BR_DRIVE_PORT);
  AHRS ahrs = new AHRS();

  public MecanumDrive mecanumDrive;
  public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV,
      Constants.kA);

  public final PIDController m_leftPIDController = new PIDController(Constants.PIDleftDrive.kP,
      Constants.PIDleftDrive.kI, Constants.PIDleftDrive.kD);
  public final PIDController m_rightPIDController = new PIDController(Constants.PIDrigthDrive.kP,
      Constants.PIDrigthDrive.kI, Constants.PIDrigthDrive.kD);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(0.125);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);
  
  /**
   * Creates a new {@link DriveTrain}.
   */
  public DriveTrain() {
    // motorFL.configFactoryDefault();
    // motorFR.configFactoryDefault();
    // motorBL.configFactoryDefault();
    // motorBR.configFactoryDefault();

    // HACK: FIx this PLSSS
    motorFL.setNeutralMode(NeutralMode.Coast);
    motorFR.setNeutralMode(NeutralMode.Coast);
    motorBL.setNeutralMode(NeutralMode.Coast);
    motorBR.setNeutralMode(NeutralMode.Coast);
    motorFL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorBL.setSelectedSensorPosition(0);
    motorBR.setSelectedSensorPosition(0);
    // motorFL.feed();
    // motorFR.feed();
    // motorBL.feed();
    // motorBR.feed();

    mecanumDrive = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);
    //mecanumDrive.setDeadband(-0.2);
    mecanumDrive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // TODO: Convert to mecanum
    SmartDashboard.putNumber("Distance left", getLeftEncoderDistance());
    SmartDashboard.putNumber("Distance right", getRightEncoderDistance());
    SmartDashboard.putNumber("Velocity left", getLeftEncoderRate());
    SmartDashboard.putNumber("Velocity right ", getLeftEncoderRate());
    putAcceleration();
  }

  /**
   * Initializes a drive mode where only one joystick controls the drive motors.
   * @param x The joystick's forward/backward tilt. Any value from -1.0 to 1.0.
   * @param y The joystick's sideways tilt. Any value from -1.0 to 1.0.
   * @param z The joystick's vertical "twist". Any value from -1.0 to 1.0.
   */
  public void singleJoystickDrive(double x, double y, double z) {
    mecanumDrive.driveCartesian(y, x, z);
  }

  public void stop() {
    mecanumDrive.stopMotor();
  }

  // NEED TO SET ALL OF THESE CORRECTLY
  public double getLeftEncoderDistance() {
    return 1.5;/*motor_left.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_REVOLUTION / Constants.DRIVE_GEAR_RATIO;*/
  }

  public double getRightEncoderDistance() {
    return 1.5;/*motor_right.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_REVOLUTION
        / Constants.DRIVE_GEAR_RATIO;*/
  }

  public double getLeftEncoderRate() {
    return 1.5;/*motor_left.getSelectedSensorVelocity()
        / (Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.DRIVE_GEAR_RATIO);*/
  }

  public double getRightEncoderRate() {
    return 1.5;/*motor_right.getSelectedSensorVelocity()
        / (Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.DRIVE_GEAR_RATIO);*/
  }

  public void putAcceleration() {
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
  }

  public void driveStraight(double power) {
    motorFL.set(power);
    motorFR.set(power);
    motorBL.set(power);
    motorBR.set(power);
  }
}
