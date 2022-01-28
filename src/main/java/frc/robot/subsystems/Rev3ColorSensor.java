// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

// import frc.robot.Constants;
public class Rev3ColorSensor extends SubsystemBase {
  ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

  /** Creates a new Rev3ColorSensor. */
  public Rev3ColorSensor() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isBallRed() {
    double red = colorSensor.getColor().red;
    if (red == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isBallBlue() {
    double blue = colorSensor.getColor().blue;
    if (blue == 1.0) {
      return true;
    } else {
      return false;
    }
  }

}