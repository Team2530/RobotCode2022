// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Rev3ColorSensor;

/**
 * This is Team 2530's Intake class.
 */
public class Intake extends SubsystemBase {
  private static WPI_TalonFX motorIntakeLower = new WPI_TalonFX(Constants.LOWER_INTAKE_PORT);
  private static WPI_TalonFX motorIntakeUpper = new WPI_TalonFX(Constants.UPPER_INTAKE_PORT);
  private final Rev3ColorSensor colorSensorUpper = new Rev3ColorSensor();
  private final Rev3ColorSensor colorSensorLower = new Rev3ColorSensor();
  double lowerIntakeSpeed = 0;
  double upperIntakeSpeed = 0;

  /** Creates a new {@link Intake}. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkChamberColors();
    lowerStallDetection();
    upperStallDetection();
  }

  /**
   * Sets the speed and direction of the intake motor.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setLowerIntakeSpeed(double speed) {
    motorIntakeLower.set(speed);
    lowerIntakeSpeed = speed;
  }

  public void setUpperIntakeSpeed(double speed) {
    motorIntakeUpper.set(speed);
    upperIntakeSpeed = speed;
  }

  public String lowerChamberColor() { 
    if (colorSensorLower.isBallRed() == true) {
      return "Red";
    } else if (colorSensorLower.isBallBlue() == true) {
      return "Blue";
    } else {
      return "Empty";
    }
  }

  public String upperChamberColor() { 
    if (colorSensorUpper.isBallRed() == true) {
      return "Red";
    } else if (colorSensorUpper.isBallBlue() == true) {
      return "Blue";
    } else {
      return "Empty";
    }
  }

  public void checkChamberColors() {
    SmartDashboard.putString("Lower Chamber", lowerChamberColor());
    SmartDashboard.putString("Upper Chamber", upperChamberColor());
  }

  // These might need either a sign change or absolute value to account for motors moving opposite directions
  public void lowerStallDetection() {
    if ((motorIntakeLower.getMotorOutputPercent() < lowerIntakeSpeed)) {
      setLowerIntakeSpeed(0);
      System.out.println("The lower intake has stopped due to a stalling issue.");
    }
  }

  public void upperStallDetection() {
    if ((motorIntakeUpper.getMotorOutputPercent() < upperIntakeSpeed)) {
      setUpperIntakeSpeed(0);
      System.out.println("The upper intake has stopped due to a stalling issue.");
    }
  }

}
