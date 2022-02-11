// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static WPI_TalonSRX climberMotorL = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_PORT_L);
  private static WPI_TalonSRX climberMotorR = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_PORT_R);

  /** Creates a new Climber. */
  public Climber() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkLimitSwitch();
  }

  private static double clamp(double f, double mi, double ma) {
    return (f < mi) ? mi : ((f > ma) ? ma : f);
  }

  /**
   * Sets the speed and direction of the climber motors.
   * 
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setClimberSpeed(double speed) {
    climberMotorL.set(clamp(-speed, -1.0, 0.0));
    climberMotorR.set(clamp(-speed, -1.0, 0.0));
  }

  public void checkLimitSwitch() {
    if ((climberMotorL.isFwdLimitSwitchClosed() == 0) || (climberMotorR.isFwdLimitSwitchClosed() == 0)) {
      climberMotorL.set(0);
      climberMotorR.set(0);
    }
  }
}
