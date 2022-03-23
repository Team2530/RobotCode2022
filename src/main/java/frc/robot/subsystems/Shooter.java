// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.Rev3ColorSensor;

/**
 * This is Team 2530's Shooter class.
 */
public class Shooter extends SubsystemBase {
  private static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_PORT);

  /** Creates a new {@link Intake}. */
  public Shooter() {
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  /**
   * Sets the speed and direction of the shooter motor.
   * 
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }
}
