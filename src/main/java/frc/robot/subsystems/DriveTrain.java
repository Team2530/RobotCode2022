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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  WPI_TalonFX motor_left = new WPI_TalonFX(Constants.motor_left_drive_port);
  WPI_TalonFX motor_right = new WPI_TalonFX(Constants.motor_right_drive_port);
  WPI_TalonSRX motor_temp = new WPI_TalonSRX(Constants.motor_revolver_port);
  AHRS ahrs = new AHRS();

  public DifferentialDrive differentialDrive;
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
    motor_left.setInverted(true);
    motor_right.setInverted(false);

    motor_left.setNeutralMode(NeutralMode.Brake);
    motor_right.setNeutralMode(NeutralMode.Brake);
    motor_left.setSelectedSensorPosition(0);
    motor_right.setSelectedSensorPosition(0);
    motor_left.feed();
    motor_right.feed();

    differentialDrive = new DifferentialDrive(motor_left, motor_right);
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
    differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance left", getLeftEncoderDistance());
    SmartDashboard.putNumber("Distance right", getRightEncoderDistance());
    SmartDashboard.putNumber("Velocity left", getLeftEncoderRate());
    SmartDashboard.putNumber("Velocity right ", getLeftEncoderRate());
    putAcceleration();
  }

  /**
   * Initializes a drive mode where only one joystick controls the drive motors.
   * @param x The joystick's forward/backward tilt. Any value from -1.0 to 1.0.
   * @param z The joystick's vertical "twist". Any value from -1.0 to 1.0.
   */
  public void singleJoystickDrive(double x, double z) {
    differentialDrive.arcadeDrive(x, z);
  }

  /**
   * Initializes a drive mode where one joystick controls each side of the robot.
   * @param left The left joystick's forward/backward tilt.
   * @param right The right joystick's forward/backward tilt.
   */
  public void dualJoystickDrive(double left, double right) {
    differentialDrive.tankDrive(left, right);
  }

  /**
   * Sets the voltages of the drive motors for each side of the robot.
   * @param leftVolts The voltage for the left side of the robot.
   * @param rightVolts The voltage for the right side of the robot.
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motor_left.setVoltage(leftVolts);
    motor_right.setVoltage(-rightVolts);
    differentialDrive.feed();
  }

    /**
   * Automatically turns the robot a specific amount of degrees.
   * Mostly experimental.
   * @param deg2Turn The amount of degrees to turn.
   */
  public void autoTurn(double deg2Turn) {
    double startRot = ahrs.getRotation2d().getDegrees();
    double endRot = startRot + deg2Turn;
    double currentRot = startRot;
    double power = Constants.autoDriveMaxTurnSpeed * Constants.autoDriveMinRampTurnSpeed;
    while (Math.abs(endRot - currentRot) > Constants.autoDriveTurnTolerance) {
      motor_left.set(-power * Constants.autoDriveMaxTurnSpeed * (endRot - currentRot) / Math.abs(endRot - currentRot));
      motor_right.set(power * Constants.autoDriveMaxTurnSpeed * (endRot - currentRot) / Math.abs(endRot - currentRot));
      currentRot = ahrs.getRotation2d().getDegrees();
      power = Constants.autoDriveMaxTurnSpeed * (Constants.autoDriveMinRampTurnSpeed + 1
          - Math.abs(((1 + Constants.autoDriveRampTurnOffset) * deg2Turn / 2) - (currentRot - startRot))
              / (deg2Turn / 2));
    }
  }

  public void stop() {
    differentialDrive.stopMotor();
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    SmartDashboard.putString("LeftEncoderRate", Double.toString(getLeftEncoderRate()));
    SmartDashboard.putString("RightEncoderRate", Double.toString(getRightEncoderRate()));

    final double leftOutput = m_leftPIDController.calculate(getLeftEncoderRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(getRightEncoderRate(), speeds.rightMetersPerSecond);
    SmartDashboard.putString("LeftVoltage", Double.toString(leftOutput + leftFeedforward));
    SmartDashboard.putString("RighttVoltage", Double.toString(rightOutput + rightFeedforward));
    motor_left.setVoltage(leftOutput + leftFeedforward);
    motor_right.setVoltage(rightOutput + rightFeedforward);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderDistance(), getRightEncoderDistance());
  }

  // NEED TO SET ALL OF THESE CORRECTLY
  public double getLeftEncoderDistance() {
    return motor_left.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_REVOLUTION / Constants.DRIVE_GEAR_RATIO;
  }

  public double getRightEncoderDistance() {
    return motor_right.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_REVOLUTION
        / Constants.DRIVE_GEAR_RATIO;
  }

  public double getLeftEncoderRate() {
    return motor_left.getSelectedSensorVelocity()
        / (Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.DRIVE_GEAR_RATIO);
  }

  public double getRightEncoderRate() {
    return motor_right.getSelectedSensorVelocity()
        / (Constants.ENCODER_TICKS_PER_REVOLUTION * Constants.DRIVE_GEAR_RATIO);
  }

  public double getAvgEncoderRate() {
    return (getLeftEncoderRate() + getRightEncoderRate()) / 2;
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
    motor_left.set(power);
    motor_right.set(power);
  }
}
