// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chambers.BallState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

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

  Timer timer = new Timer();

  // -----------Intake Behavior States-----------\\
  SimpleWidget lowerIntakeWidget;
  SimpleWidget upperIntakeWidget;
  // ball was rejected by the robot
  boolean ballRejection = false;
  // upper chamber is currently empty
  boolean upperChamberEmpty = true;
  // The rejection button is being pressed
  boolean reverseIsPressed = true;
  // you have dumped the ball that the robot rejected
  boolean reverseWasPressed = true;
  // a ball may be eaten
  boolean readyToIntake = true;
  // a ball is being consumed
  boolean intaking = false;
  // robot has 2 balls
  boolean robotFull = false;
  // Robot is full and able to shoot
  boolean readyToShoot = false;
  // shooter is being used
  boolean shooting = false;

  // Current state of auto-intake
  String autoIntakeDescription = "Not run yet";

  double timerLast = 0.0;
  int executed = 0;

  Joystick stick = new Joystick(Constants.stickport1);
  XboxController xbox = new XboxController(Constants.xboxport);

  /** Creates a new {@link Intake}. */
  public Intake() {
    inputSpeeds[0] = 0;
    inputSpeeds[1] = 0;
    intakeMotorSpeeds[0] = 0;
    intakeMotorSpeeds[1] = 0;
    upperIntakeWidget = Shuffleboard.getTab("Driver Dashboard").add("Upper intake status", true);
    lowerIntakeWidget = Shuffleboard.getTab("Driver Dashboard").add("Lower intake status", true);
    Shuffleboard.getTab("Technical Info").add("Current intake auto", autoIntakeDescription);
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
        reverseWasPressed = false;
        readyToIntake = false;
        autoIntakeDescription = "Ball rejection";
      } else if ((Chambers.states[0] == matchingBallState || Chambers.states[1] == matchingBallState
          || Chambers.states[2] == matchingBallState)
          && Chambers.ballNotDetected[3]) {
        // Move lower chamber contents up if upper chamber is empty
        intakeMotorSpeeds[0] = -Constants.intakeSpeed;
        intakeMotorSpeeds[1] = -Constants.intakeSpeed;
        autoIntakeDescription = "Upper chamber empty";
        upperChamberEmpty = true;
      } else if ((Chambers.ballDetected[0] || Chambers.ballDetected[1])
          && Chambers.ballDetected[3]) {
        // Do not allow running bottom intake up if both chambers are full
        intakeMotorSpeeds[0] = inputSpeeds[0] < 0 ? 0 : inputSpeeds[0];
        intakeMotorSpeeds[1] = inputSpeeds[1];
        autoIntakeDescription = "full";
      } else if (Chambers.ballDetected[1]) {
        // Chamber transfer - run both intake motors in the direction of the lower
        // intake
        intakeMotorSpeeds[0] = inputSpeeds[0];
        intakeMotorSpeeds[1] = inputSpeeds[0];
        autoIntakeDescription = "Chamber transfer";
      } else {
        // Standard intake behavior
        intakeMotorSpeeds[0] = inputSpeeds[0];
        intakeMotorSpeeds[1] = inputSpeeds[1];
        autoIntakeDescription = "Normal";
      }
      // Do not move upper intake if a ball is present and the shooter is running but
      // is below the allowed speed threshold
      if (Chambers.ballDetected[3] && !Shooter.meetsSpeedThreshold()) {
        intakeMotorSpeeds[1] = 0;
      }
    } else {
      // Standard intake behavior
      intakeMotorSpeeds[0] = inputSpeeds[0];
      intakeMotorSpeeds[1] = inputSpeeds[1];
      autoIntakeDescription = "Manual mode";
    }
    // SmartDashboard.putString("inputSpeeds", inputSpeeds[0] + " " +
    // inputSpeeds[1]);
    // SmartDashboard.putString("actualSpeeds", intakeMotorSpeeds[0] + " " +
    // intakeMotorSpeeds[1]);
    intakeSpeedGradient();

    // Update Shuffleboard panels
    if (xbox.getRawButton(2)) {
      reverseWasPressed = true;
      readyToIntake = true;
    }
    intaking = xbox.getAButton();
    reverseIsPressed = xbox.getRawButton(2);
    shooting = xbox.getRawButton(3);

    readyToShoot = Chambers.ballDetected[2] || Chambers.ballDetected[3];

    if (autoIntakeDescription == "Ball rejection" && !reverseWasPressed) {
      readyToIntake = false;
      setIntakeStatus(lowerIntakeWidget, flashColor("yellow", "white", 15));
    } else if (reverseIsPressed) {
      setIntakeStatus(lowerIntakeWidget, "yellow");
    } else if (intaking) {
      setIntakeStatus(lowerIntakeWidget, "green");
    } else if (readyToIntake) {
      setIntakeStatus(lowerIntakeWidget, flashColor("green", "white", 20));
    } else if (Chambers.ballDetected[0] || Chambers.ballDetected[1]) {
      setIntakeStatus(lowerIntakeWidget, "#b3b3b3");
    } else {
      setIntakeStatus(lowerIntakeWidget, "white");
    }

    if (shooting) {
      setIntakeStatus(upperIntakeWidget, "green");
    } else if (readyToShoot) {
      setIntakeStatus(upperIntakeWidget, flashColor("green", "white", 20));
    } else if (autoIntakeDescription == "Upper chamber empty") {
      setIntakeStatus(upperIntakeWidget, flashColor("#b3b3b3", "white", 20));
    } else if ((Chambers.states[0] == matchingBallState || Chambers.states[1] == matchingBallState)
        && (Chambers.states[2] == matchingBallState
            || Chambers.states[3] == matchingBallState)) {
      setIntakeStatus(upperIntakeWidget, "#b3b3b3");
    } else {
      setIntakeStatus(upperIntakeWidget, "white");
    }

    // Ticks for flashing
    executed++;
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

  public void setIntakeStatus(SimpleWidget intakeWidget, String color) {
    intakeWidget.withProperties(Map.of("colorWhenTrue", color));
  }

  public String flashColor(String color1, String color2, int ticks) {
    if (executed > ticks * 2)
      executed = 0;
    if (executed < ticks) {
      return color1;
    } else {
      return color2;
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
