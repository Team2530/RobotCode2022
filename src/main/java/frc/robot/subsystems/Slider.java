// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slider extends SubsystemBase {
  //---------------How to create a Slider------------------\\
  // NetworkTableEntry example = Shuffleboard.getTab("My Tab")
  // .add("My Number", 0)
  // .withWidget(BuiltInWidgets.kNumberSlider)
  // .withProperties(Map.of("min", 0, "max", 1))
  // .getEntry();

  // Slider Instances
  NetworkTableEntry rotPidP;
  NetworkTableEntry rotPidI;
  NetworkTableEntry rotPidD;


  // Slider Values
  static double ROT_PID_P = 0;
  static double ROT_PID_I = 0;
  static double ROT_PID_D = 0;
  /** Creates a new Slider. */
  public Slider() {
    rotPidP = Shuffleboard.getTab("PID Constants").add("Rot P", 0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    rotPidI = Shuffleboard.getTab("PID Constants").add("Rot I", 0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    rotPidD = Shuffleboard.getTab("PID Constants").add("Rot D", 0).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ROT_PID_P = rotPidP.getDouble(ROT_PID_P);
    ROT_PID_I = rotPidI.getDouble(ROT_PID_I);
    ROT_PID_D = rotPidD.getDouble(ROT_PID_D);
    System.out.println(ROT_PID_P);
    System.out.println(ROT_PID_I);
    System.out.println(ROT_PID_D);
    
  }
}
