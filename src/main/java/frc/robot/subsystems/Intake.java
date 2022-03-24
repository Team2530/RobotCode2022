// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.Chambers.BallState;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.Rev3ColorSensor;

/**
 * This is Team 2530's Intake class.
 */
public class Intake extends SubsystemBase {
  private static WPI_TalonFX[] intakeMotors = {
      new WPI_TalonFX(Constants.LOWER_INTAKE_PORT),
      new WPI_TalonFX(Constants.UPPER_INTAKE_PORT)
  };

  /** The target expected motor speeds. */
  private static double[] intakeMotorSpeeds = { 0, 0 };

  /** Creates a new {@link Intake}. */
  public Intake() {
    intakeMotorSpeeds[0] = 0;
    intakeMotorSpeeds[1] = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // stallDetection();
    intakeSpeedGradient();
  }

  /**
   * Sets the speed and direction of the intake motor.
   * 
   * @param speed Any value from -1.0 to 1.0, with negative moving the balls up.
   */
  public void setIntakeMotorSpeed(int idx, double speed) {
    BallState matchingBallState = DriverStation.getAlliance() == DriverStation.Alliance.Red
        ? BallState.Red
        : BallState.Blue;
    BallState opposingBallState = matchingBallState == BallState.Red
        ? BallState.Blue
        : BallState.Red;
    if (Chambers.ballDetected[1] || Chambers.ballDetected[2]) {
      // Chamber transfer
      intakeMotorSpeeds[0] = speed;
      intakeMotorSpeeds[1] = speed;
    } else if (Chambers.states[0] == opposingBallState || Chambers.states[1] == opposingBallState) {
      // Ball rejection
      intakeMotorSpeeds[0] = Math.abs(speed);
      if (idx == 1) {
        intakeMotorSpeeds[1] = speed;
      }
    } else if ((Chambers.states[0] == matchingBallState || Chambers.states[1] == matchingBallState)
        && Chambers.ballNotDetected[2] && Chambers.ballNotDetected[3]) {
      // Automatic chamber transport
      intakeMotorSpeeds[0] = -Math.abs(speed);
      intakeMotorSpeeds[1] = -Math.abs(speed);
    } else {
      // Standard intake behavior
      intakeMotorSpeeds[idx] = speed;
    }
  }

  public void intakeSpeedGradient() {
    // Do for each intake motor
    for (int motor = 0; motor < 2; ++motor) {
      /*
       * Calculate difference between target expected motor speed and current expected
       * motor speed
       */
      if (Math.abs(intakeMotorSpeeds[motor] - intakeMotors[motor].get()) > Constants.INTAKE_RAMP_INTERVAL) {
        // If we're not there yet
        intakeMotors[motor].set(
            intakeMotors[motor].get()
                + Constants.INTAKE_RAMP_INTERVAL * Math.signum(intakeMotorSpeeds[motor] - intakeMotors[motor].get()));
      } else {
        // If our patience has paid off
        intakeMotors[motor].set(intakeMotorSpeeds[motor]);
      }
    }
  }

  /*
   * public void stallDetection() {
   * for (int i = 0; i < 2; ++i) {
   * if ((intakeMotors[i].getMotorOutputPercent()) <
   * (Math.abs(intakeMotorSpeeds[i] * 60))) {
   * setIntakeMotorSpeed(i, 0);
   * System.out.println("The lower intake has stopped due to a stalling issue.");
   * }
   * }
   * }
   */

}
