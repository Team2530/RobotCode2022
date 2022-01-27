// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static WPI_TalonFX motor_Climber = new WPI_TalonFX(Constants.motor_climber_port);
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed and direction of the intake motor.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setClimberSpeed(double speed) {
    motor_Climber.set(speed);
    // This method will be called once per scheduler run

  }
}
