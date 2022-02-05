// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static WPI_TalonSRX climberMotorL = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_PORT_L);
  private static WPI_TalonSRX climberMotorR = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_PORT_R);
  private static DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkLimitSwitch();
  }

  /**
   * Sets the speed and direction of the climber motors.
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setClimberSpeed(double speed) {
      if ((limitSwitch.get() == false) && (speed > 0)) {
        climberMotorL.set(speed);
        climberMotorR.set(speed);
      }  
    }

  public void checkLimitSwitch() {
    if (limitSwitch.get() == true) {
      setClimberSpeed(0);
    }
  }
}
