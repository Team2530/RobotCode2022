// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;

public class UltraSonic extends SubsystemBase {
  DriveTrain driveTrain;
  double currentDistanceInches1 = 0.0;
  double currentDistanceInches2 = 0.0;
  double distanceFromGoalStop = 0.0;
  public int runIntoSomething = 1;
  private final AnalogInput ultrasonic1 = new AnalogInput(0);
  // if you use two UltraSonic Sensors
  // private final AnalogInput ultrasonic2 = new AnalogInput(1);
  /** Creates a new UltraSonic. */
  public UltraSonic() {}
  
  
  // This method will be called once per scheduler run

  @Override
  public void periodic() {}

public void ultraSonicBoom(){
  double raw_value1 = ultrasonic1.getValue();
  // double raw_value2 = ultrasonic2.getValue();
  // voltage_scale_factor allows us to compensate for differences in supply voltage.
  double voltage_scale_factor = 5 / RobotController.getVoltage5V();
  double currentDistanceInches1 = raw_value1 * voltage_scale_factor * 0.0492;
  // double currentDistanceInches2 = raw_value2 * voltage_scale_factor * 0.0492;

  if (currentDistanceInches1 <= 60) {
      runIntoSomething = 0;
    } else {
      runIntoSomething = 1;
    }
  }
}
