// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is Team2530's Hood class. It handles all things related to the shooter
 * component of the robot, including the flywheel and horizontal/vertical
 * aiming.
 */
public class Hood extends SubsystemBase {
  private static WPI_TalonSRX motor_hood = new WPI_TalonSRX(Constants.motor_hood_port);
  public static WPI_TalonFX motor_flywheel = new WPI_TalonFX(Constants.motor_flywheel_port);
  private static WPI_TalonSRX motor_turret = new WPI_TalonSRX(Constants.motor_turret_port);
  /** Creates a new Hood. */
  double flywheelSpeed;
  double hoodPosition = 0; // TODO: Make more reliable method of measurement
  double angleCalc;
  double velocityX;
  double velocityY;
  double timeCalc;
  double aimHeightCalc;
  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light = 1;
  int camMode = 0;
  public double hoodTargetAngle = 0;
  double hoodPos = 0.5;
  boolean targeting = false;

  /** Creates a new {@link Hood}. */
  public Hood() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    motor_flywheel.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    motor_flywheel.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    motor_flywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    motor_flywheel.configNominalOutputForward(0, Constants.kTimeoutMs);
    motor_flywheel.configNominalOutputReverse(0, Constants.kTimeoutMs);
    motor_flywheel.configPeakOutputForward(1, Constants.kTimeoutMs);
    motor_flywheel.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    motor_flywheel.config_kF(Constants.kPIDLoopIdx, Constants.motor_Shooter.kF, Constants.kTimeoutMs);
    motor_flywheel.config_kP(Constants.kPIDLoopIdx, Constants.motor_Shooter.kP, Constants.kTimeoutMs);
    motor_flywheel.config_kI(Constants.kPIDLoopIdx, Constants.motor_Shooter.kI, Constants.kTimeoutMs);
    motor_flywheel.config_kD(Constants.kPIDLoopIdx, Constants.motor_Shooter.kD, Constants.kTimeoutMs);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putNumber("Hood pos", hoodPos);
    light = (int) table.getEntry("ledMode").getDouble(0.0);
    camMode = (int) table.getEntry("camMode").getDouble(0.0);
  }

  @Override
  public void periodic() {
    if (targeting) {
      if (tx > 3) {
        motor_turret.set(ControlMode.PercentOutput, -1);
      } else if (tx < -3) {
        motor_turret.set(ControlMode.PercentOutput, 1);
      } else {
        motor_turret.stopMotor();
      }
    } else {
      motor_turret.stopMotor();
    }
    // This method will be called once per scheduler run

    // TODO: Reimplement Limelight hood aiming
    // if (hoodPosition < Constants.MIN_SHOOTING_ANGLE + 1 || hoodPosition >
    // Constants.MAX_SHOOTING_ANGLE - 1 || (hoodPosition > hoodTargetAngle - 1 &&
    // hoodPosition < hoodTargetAngle + 1)) {
    // setHoodPosition(0);
    // hoodTargetAngle = 0;
    // }
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(light);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
    SmartDashboard.putNumber("tx", tx);

    // moveHoodToAngle(ty);
  }

  /** Switches the setting for the Limelight aiming mechanism. */
  public void toggleAim() {
    targeting = !targeting;
  }

  /**
   * Automatically aims the hood using the Limelight.
   * @param distance The distance reading from the Limelight.
   */
  public void autoAimHood(double distance) { // ! everything must stay in meters
    angleCalc = Math.atan((Constants.target_Height - Constants.SHOOTER_HEIGHT) / distance);
    velocityX = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.cos(angleCalc);
    velocityY = flywheelSpeed * Constants.MAX_SHOOTING_VELOCITY * Math.sin(angleCalc);
    timeCalc = distance / velocityX;
    aimHeightCalc = (velocityY * timeCalc) + (0.5 * Constants.gravity * Math.pow(timeCalc, 2));
    moveHoodToAngle(Math.atan((2 * Constants.target_Height - aimHeightCalc) / distance));
  }

  /**
   * Moves the hood vertically to the specified angle.
   * @param angle The angle, in degrees.
   */
  public void moveHoodToAngle(double angle) {
    // 52/0.2

    double pos = 0.6 - ((angle) / 300);
    motor_hood.set(pos); // TODO: Needs to be replaced with SRX-friendly encoder method
  }

  /**
   * Sets the speed and direction of the vertical hood aiming motor.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setHood(double speed) {
    motor_hood.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the speed and direction of the flywheel motor.
   * @param f_speed Any value from -1.0 to 1.0.
   */
  public void flywheelRotateSpeed(double f_speed) {
    motor_flywheel.set(ControlMode.PercentOutput, f_speed);
  }

  /**
   * Sets the speed and direction of the horizontal hood aiming motor.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setTurretPower(double speed) {
    motor_turret.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Switches the setting of the Limelight LEDs.
   */
  public void toggleLight() {
    switch (light) {
    case 1:
      light = 3;
      break;
    case 3:
      light = 1;
      break;
    default:
      light = 3;
      break;
    }

  }
  public void toggleCamMode() {
    switch (camMode) {
    case 0:
      camMode = 1;
      break;
    case 1:
      camMode = 0;
      break;
    default:
      camMode = 0;
      break;
    }

  }

}