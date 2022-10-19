// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.libraries.Deadzone;

public class LimeLight extends SubsystemBase {
  static DriveTrain driveTrain;

  static double xoff;
  static double yoff;
  static double area;
  /** Any targets? */
  static double tv;

  static double turnRate;

  /** Gain for turning for the LimeLight */
  static double limekP = 0.012;
  static double limekI = 0.001;
  static double limekD = 0.000;
  static PIDController greenPid = new PIDController(limekP, limekI, limekD);

  /** If the turn value is really low, we add to it so it still moves */
  static double minCommand = 0.07;
  /** Amount we are willing to compromise for in our distance */
  static double disttolerance = 0.9;

  // not sure if this is right or not
  static int lightMode = 3;

  int cameraMode = 0;

  static Joystick stick;

  /** Creates a new LimeLight. */
  public LimeLight(DriveTrain driveTrain, Joystick stick) {
    this.driveTrain = driveTrain;
    this.stick = stick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateValues();
    showValues();
    double distanceFromTarget = distanceToTarget();
    // System.out.println(distanceFromTarget);
    // Shuffleboard.getTab("limelight").addNumber("Distance", distanceFromTarget);
    SmartDashboard.putNumber("Lime Distance", distanceFromTarget);
  }

  /**
   * Updates the LimeLight's values
   */
  public static void updateValues() {
    xoff = getLimeValues("targetYaw");
    System.out.println(xoff);
    yoff = getLimeValues("ty");
    // tv = table.getEntry("tv").getDouble(0.0);
    area = getLimeValues("ta");
  }

  public void showValues() {
    SmartDashboard.putNumber("LimelightX", xoff);
    SmartDashboard.putNumber("LimelightY", yoff);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  public double toRadians(double input) {
    return input * (Math.PI / 180.0);
  }

  public double distanceToTarget() {
    double r = toRadians(Constants.limeAngle + yoff);
    return (Constants.goalHeight - Constants.limeHeight) / Math.tan(r);
  }

  public void changeMode() {
    if (lightMode == 3) {
      lightMode = 1;
    } else {
      lightMode = 3;
    }
  }

  /**
   * Assume that there is a valid target, we will turn to aim at it
   */
  public static void aimAtTarget() {
    double error = -xoff;
    // if' (stick.getMagnitude() < 0.2) return;
    double m = RobotContainer.getBoostMode() ? 1.0 : 0.5;
    double s = RobotContainer.getSlowMode() ? 0.5 : 1;
    // m *= (stick.getRawAxis(3) + 1.0) / 2.0;

    // double turn = stick.getRawAxis(3) - stick.getRawAxis(2);

    // m_drivetrain.singleJoystickDrive(stick.getX() * m, 0, 0);

    if (xoff < 0) {
      turnRate = limekP * error + minCommand;
    } else {
      turnRate = limekP * error - minCommand;
    }
    System.out.println(turnRate);
    // Use this method to turn to robot at the speeds
    // driveTrain.setSides(turnRate, -turnRate);
    driveTrain.singleJoystickDrive(Deadzone.deadZone(stick.getRawAxis(1), Constants.deadzone) * m * s,
        Deadzone.deadZone(stick.getRawAxis(0), Constants.deadzone) * m * s,
        -turnRate);
  }

  public void fixOffset() {
    double error = -xoff;
    // driveTrain.deathBlossom(error);
    double driveangle = driveTrain.ahrs.getAngle();
    driveTrain.yawTarget = driveangle;
  }

  /**
   * Backs up to a given distance based on maths
   * <p>
   * Meant to be used called multiple times whilst preparing to shooting is
   * occuring
   * 
   * @param dist Distance away from goal
   */
  public void backToDistance(double dist) {
    double currentdist = distanceToTarget();

  }

  /**
   * Gets the different values from NetworkTables limelight
   * 
   * @param tvar String of the t value you want (ta , tx , ty , etc.)
   */

  public static double getLimeValues(String tvar) {
    return NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("Limelight").getEntry(tvar)
        .getDouble(0.0);
  }

}