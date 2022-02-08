// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.subsystems.BallDetection.BallState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.subsystems.Rev3ColorSensor;

/**
 * This is Team 2530's Intake class.
 */
public class Intake extends SubsystemBase {
  private static WPI_TalonSRX[] intakeMotors = {
      new WPI_TalonSRX(Constants.LOWER_INTAKE_PORT),
      new WPI_TalonSRX(Constants.UPPER_INTAKE_PORT)
  };

  private static double[] intakeMotorSpeeds = { 0, 0 };

  /** Creates a new {@link Intake}. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // lowerStallDetection();
    // stallDetection();
    removeBall();
    // intakeSpeedGradient();
  }

  /**
   * Sets the speed and direction of the intake motor.
   * 
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setIntakeMotorSpeed(int idx, double speed) {
    intakeMotors[idx].set(speed);
    intakeMotorSpeeds[idx] = speed;
  }

  // public void intakeSpeedGradient() {
  // for (int g = 0; g < 2; ++g) {
  // if (intakeMotorSpeeds[g] - intakeMotors[g].get() > 0.1) {
  // // figuring things out
  // intakeMotors[g].set(intakeMotors[g].get() + 0.1 *
  // Math.signum(intakeMotorSpeeds[g] - intakeMotors[g].get()));
  // } else
  // intakeMotors[g].set(intakeMotorSpeeds[g]);
  // }
  // }

  // These might need a sign change/absolute value to account for motors moving
  // opposite directions
  // public void stallDetection() {
  // for (int i = 0; i < 2; ++i) {
  // if (intakeMotors[i].getMotorOutputPercent() < (intakeMotorSpeeds[i] * 60)) {
  // setIntakeMotorSpeed(i, 0);
  // System.out.println("The lower intake has stopped due to a stalling issue.");
  // }
  // }
  // }

  // Needs a stop condition
  // Also possibly gradient speed from current to backwards?
  // Might cause issues if trying to drive intake motors as this is running
  public void removeBall() {
    for (int i = 0; i < 2; ++i) {
      if (((DriverStation.getAlliance() == Alliance.Red) ? BallState.Blue : BallState.Red) == BallDetection.states[i]) {
        setIntakeMotorSpeed(i, 1.0);
      }
    }
  }
}
