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
 * This is Team 2530's Intake class. Its only method, `setIntakeSpeed`, is able
 * to control the speed of the intake motor.
 */
public class Intake extends SubsystemBase {
  private static WPI_TalonFX motorIntakeLower = new WPI_TalonFX(Constants.LOWER_INTAKE_PORT);
  private static WPI_TalonFX motorIntakeUpper = new WPI_TalonFX(Constants.UPPER_INTAKE_PORT);
  private final Rev3ColorSensor colorSensorUpper = new Rev3ColorSensor();
  private final Rev3ColorSensor colorSensorLower = new Rev3ColorSensor();

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
  public void setLowerIntakeSpeed(double speed) {
    motorIntakeLower.set(speed);
  }

  public void setUpperIntakeSpeed(double speed) {
    motorIntakeUpper.set(speed);
  }
  
  /*stalling detection */
  public void WPI_TalonFX(double angle) {
    double prevPos = motorIntakeLower.getSelectedSensorPosition();
    try {
      motorIntakeLower.setSelectedSensorPosition(prevPos + (angle / 360 * Constants.ENCODER_TICKS_PER_REVOLUTION), 0,
          1000);
    } catch (Exception e) {
      setLowerIntakeSpeed(0);
      System.out.println("The lower intake stopped because it detected a stalling issue.");
    }
    prevPos = motorIntakeUpper.getSelectedSensorPosition();
    try {
      motorIntakeUpper.setSelectedSensorPosition(prevPos + (angle / 360 * Constants.ENCODER_TICKS_PER_REVOLUTION), 0,
          1000);
    } catch (Exception e) {
      setUpperIntakeSpeed(0);
      System.out.println("The upper intake stopped because it detected a stalling issue.");
    }
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

  public void checkChamberColors(String lowerChamberColor, String upperChamberColor) {
    SmartDashboard.putString("Lower Chamber", lowerChamberColor);
    SmartDashboard.putString("Upper Chamber", upperChamberColor);
  }
}
