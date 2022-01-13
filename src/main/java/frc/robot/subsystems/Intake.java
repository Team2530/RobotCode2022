// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

/**
 * This is Team 2530's Intake class. Its only method, `setIntakeSpeed`, is able
 * to control the speed of the intake motor.
 */
public class Intake extends SubsystemBase {
  private static WPI_VictorSPX motor_Intake = new WPI_VictorSPX(Constants.motor_intake_port);

  /** Creates a new {@link Intake}. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed and direction of the intake motor.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setIntakeSpeed(double speed) {
    motor_Intake.set(speed);
  }
}
