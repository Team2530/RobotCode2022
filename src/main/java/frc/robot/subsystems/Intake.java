// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chambers.BallState;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.Rev3ColorSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is Team 2530's Intake class.
 */
public class Intake extends SubsystemBase {
  private static WPI_TalonFX[] intakeMotors = {
      new WPI_TalonFX(Constants.LOWER_INTAKE_PORT),
      new WPI_TalonFX(Constants.UPPER_INTAKE_PORT)
  };

  /** The inputted motor speeds. */
  private static double[] inputSpeeds = { 0, 0 };
  /** The target expected motor speeds. */
  private static double[] intakeMotorSpeeds = { 0, 0 };

  /** Creates a new {@link Intake}. */
  public Intake() {
    inputSpeeds[0] = 0;
    inputSpeeds[1] = 0;
    intakeMotorSpeeds[0] = 0;
    intakeMotorSpeeds[1] = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BallState matchingBallState = DriverStation.getAlliance() == DriverStation.Alliance.Red
        ? BallState.Red
        : BallState.Blue;
    BallState opposingBallState = matchingBallState == BallState.Red
        ? BallState.Blue
        : BallState.Red;
    if (!RobotContainer.getManualModeOp()) {
      if (Chambers.states[0] == opposingBallState || Chambers.states[1] == opposingBallState) {
        // Ball rejection - run bottom intake down, run top intake as usual
        intakeMotorSpeeds[0] = Constants.intakeSpeed;
        intakeMotorSpeeds[1] = inputSpeeds[1];
        SmartDashboard.putString("Current intake auto", "ball rejection");
      } else if ((Chambers.states[0] == matchingBallState || Chambers.states[1] == matchingBallState
          || Chambers.states[2] == matchingBallState)
          && Chambers.ballNotDetected[3]) {
        // Move lower chamber contents up if upper chamber is empty
        intakeMotorSpeeds[0] = -Constants.intakeSpeed;
        intakeMotorSpeeds[1] = -Constants.intakeSpeed;
        SmartDashboard.putString("Current intake auto", "upper chamber empty");
      } else if (Chambers.ballDetected[1]) {
        // Chamber transfer - run both intake motors in the direction of the lower
        // intake
        intakeMotorSpeeds[0] = inputSpeeds[0];
        intakeMotorSpeeds[1] = inputSpeeds[0];
        SmartDashboard.putString("Current intake auto", "chamber transfer");
      } else {
        // Standard intake behavior
        intakeMotorSpeeds[0] = inputSpeeds[0];
        intakeMotorSpeeds[1] = inputSpeeds[1];
        SmartDashboard.putString("Current intake auto", "normal");

      }
    } else {
      // Standard intake behavior
      intakeMotorSpeeds[0] = inputSpeeds[0];
      intakeMotorSpeeds[1] = inputSpeeds[1];
      SmartDashboard.putString("Current intake auto", "normal");

    }
    SmartDashboard.putString("inputSpeeds", inputSpeeds[0] + " " + inputSpeeds[1]);
    SmartDashboard.putString("actualSpeeds", intakeMotorSpeeds[0] + " " + intakeMotorSpeeds[1]);
    intakeSpeedGradient();
  }

  /**
   * Sets the speed and direction of the input for an intake motor.
   * 
   * @param idx   0 for bottom chamber, 1 for top chamber
   * @param speed Any value from -1.0 to 1.0, with negative moving the balls up.
   */
  public void setIntakeMotorSpeed(int idx, double speed) {
    inputSpeeds[idx] = speed;
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
