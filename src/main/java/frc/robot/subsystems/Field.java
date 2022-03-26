// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class Field extends SubsystemBase {
  /** Creates a new Field2d. */

    // --------------------Field2d Stuff------------------------\\
    AHRS ahrs;
    DriveTrain driveTrain;
    Field2d m_field = new Field2d();
    Pose2d m_pose = new Pose2d();
    Rotation2d m_rotation = new Rotation2d();
    Rotation2d rotation;
    double fieldXPos = 8.0;
    double fieldYPos = 4.0;
    double fieldRotation = 0.0;

    double fieldSpeed[] = { 0.0, 0.0, 0.0 };
  public Field() {
    SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field2d(fieldXPos, fieldYPos, fieldRotation);
    // field2dSimuationMode();
  }

  public void field2d(double fieldXPos, double fieldYPos, double fieldRotation) {
    this.fieldXPos = fieldXPos;
    this.fieldYPos = fieldYPos;
    this.fieldRotation = fieldRotation;
    m_field.setRobotPose(fieldXPos, fieldYPos, m_rotation);
    // NavX Rotated 90 (Values below should update realtime robot position)
    fieldXPos = fieldXPos + ahrs.getVelocityY();
    fieldYPos = fieldYPos + ahrs.getVelocityX();
    fieldRotation = fieldRotation + ahrs.getVelocityZ();  
  }

/**
 * Makes sure the robot dosen't go off screen
 */
  public void field2dBounds() {
    if (fieldXPos > 16) {
      fieldXPos = 16;
    }
    if (fieldXPos < 0) {
      fieldXPos = 0;
    }
    if (fieldYPos < 0) {
      fieldYPos = 0;
    }
    if (fieldYPos > 8) {
      fieldYPos = 8;
    }
  }
  /** To use when using field2d on simuation mode */
  public void field2dSimuationMode(){
    if (Math.abs(DriveTrain.joystickInput[1]) >= .1) {
      fieldSpeed[0] = (DriveTrain.joystickLerp[1] / 8);
      } else {
      fieldSpeed[0] = fieldSpeed[0] * 0.92;
    }
    fieldXPos = fieldXPos + fieldSpeed[0];
    if (Math.abs(DriveTrain.joystickInput[0]) >= .1) {
  fieldSpeed[1] = (DriveTrain.joystickLerp[0] / 8);
    } else {
      fieldSpeed[1] = fieldSpeed[1] * 0.92;
    }
    fieldYPos = fieldYPos - fieldSpeed[1];
    if (Math.abs(DriveTrain.joystickInput[2]) >= .5) {
      fieldSpeed[2] = fieldSpeed[2] + (DriveTrain.joystickLerp[2] / 3);
    } else {
      fieldSpeed[2] = fieldSpeed[2] * 0.8;
    }
    fieldRotation = fieldRotation - fieldSpeed[2];
    m_rotation = Rotation2d.fromDegrees(fieldRotation);
    field2dBounds();
    System.out.println(DriveTrain.joystickInput[0]);
    System.out.println(DriveTrain.joystickInput[1]);
    System.out.println(DriveTrain.joystickInput[2]);
  }

}
