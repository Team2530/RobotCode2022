/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /**
   * Creates a new LimeLight.
   */
  double tx;
  double ty;
  double ta;
  NetworkTable table;
  int light;
  int camMode;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setLight(1); // 1 = off, 3 = on, 2 EQUALS BLINKY
    setCamMode(1); // 1 = on, 0 = off
  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);
    SmartDashboard.putNumber("light", light);
    SmartDashboard.putNumber("camMode", light);
  }

  public void toggleLight() {
    if (light == 1) {
      setLight(3);
    } else {
      setLight(1);
    }
  }

  public void toggleCamMode() {
    if (camMode == 1) {
      setCamMode(0);
    } else {
      setCamMode(1);
    }
  }

  public void setLight(int number) {
    light = number;
    table.getEntry("ledMode").setNumber(number);
  }

  public void setCamMode(int number) {
    camMode = number;
    table.getEntry("camMode").setNumber(number);
  }
}
