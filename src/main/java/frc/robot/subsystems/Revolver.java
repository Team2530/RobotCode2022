// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is Team 2530's Revolver class. It handles all things related to the
 * ball carousel, such as setting its speed/direction and rotating by a specific
 * number of degrees.
 */
public class Revolver extends SubsystemBase {

  private static WPI_TalonFX motor_Revolver = new WPI_TalonFX(Constants.motor_revolver_port);

  /** Creates a new {@link Revolver}. */
  public Revolver() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Sets the speed of the revolver motor. 
   * @param speed Any value between -1.0 and 1.0.
  */
  public void setRevolverSpeed(double speed) {
    motor_Revolver.set(speed);
  }

  /** Rotates the revolver by the specified angle.
   * @param angle The angle, in degrees.
   */
  public void rotateRevolver(double angle) {
    double prevPos = motor_Revolver.getSelectedSensorPosition();
    try {
      motor_Revolver.setSelectedSensorPosition(prevPos + (angle / 360 * Constants.ENCODER_TICKS_PER_REVOLUTION), 0,
          1000);
    } catch (Exception e) {
      setRevolverSpeed(0);
      System.out.println("The revolver stopped because it detected a stalling issue.");
    }
  }

}
