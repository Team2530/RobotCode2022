// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.I2CLink;

public class Pixy extends SubsystemBase {
  private Pixy2 pixy;
  private boolean blockFound;
  private int x, y, width, height;
  private int fps;

  /** Creates a new Pixy. */
  public Pixy() {
    // Initialize Pixy2.
    pixy = Pixy2.createInstance(new I2CLink());
    int err = pixy.init();
    if (err < 0) {
      System.out.println("An error occurred while the roboRIO was trying to initialize the Pixy.");
      logPixyError(err);
    }

    // Print Pixy2 firmware version info.
    System.out.println("[PIXY] " + pixy.getVersionInfo());

    // Set LED and lamp to white.
    pixy.setLamp((byte) 1, (byte) 1);
    pixy.setLED(0, 255, 255);

    // Put the alliance SmartDashboard variable.
    SmartDashboard.putBoolean("Pixy/alliance", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Error codes returned from pixy2 functions.
    int err;

    fps = pixy.getFPS();

    // Get which alliance the robot is playing on,
    // and set the signature of the pixy accordingly. As of now, signature 1 is for
    // the red cargo, signature 2 is for the blue cargo.
    byte signature = DriverStation.getAlliance() == Alliance.Red ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

    // First argument is set to true to stop the PIXY_RESULT_BUSY error.
    err = pixy.getCCC().getBlocks(true, signature, 25);
    if (err <= 0) {
      blockFound = false;

      if (err < 0) {
        System.out.println("An error occurred while the roboRIO was trying to get blocks from the Pixy.");
        logPixyError(err);
      }
    } else {
      blockFound = true;

      // Get list of blocks.
      ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
      Block largestBlock = null;

      // Loop through blocks to find the one with the largest width.
      for (Block block : blocks) {
        if (largestBlock == null) {
          largestBlock = block;
        } else if (block.getWidth() > largestBlock.getWidth()) {
          largestBlock = block;
        }
      }

      x = largestBlock.getX();
      y = largestBlock.getY();
      width = largestBlock.getWidth();
      height = largestBlock.getHeight();
    }

    // Log everything to SmartDashboard
    SmartDashboard.putBoolean("Pixy/Found Block", blockFound);
    SmartDashboard.putNumber("Pixy/x", (double) x);
    SmartDashboard.putNumber("Pixy/y", (double) y);
    SmartDashboard.putNumber("Pixy/width", (double) width);
    SmartDashboard.putNumber("Pixy/height", (double) height);
    SmartDashboard.putNumber("Pixy/fps", (double) fps);
  }

  /**
   * Prints a error code from a pixy2 function.
   * 
   * @param errorCode
   */
  private void logPixyError(int errorCode) {
    String[] errors = { "The Pixy isn't connected to the robot!" };
    System.out.print("Pixy error " + errorCode);
    if (Math.abs(errorCode) - 1 < errors.length)
      System.out.println(": " + (errors[Math.abs(errorCode) - 1]));
    else
      System.out.println();
  }
}
