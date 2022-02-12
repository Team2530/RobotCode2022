// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Pixy extends TimedRobot {
  // SPI Slave Select pin (wired to CS0 on the practice chassix).
  private static final int kPixySSPort = 0;

  private Pixy2 pixy;

  private boolean blockFound;
  private int x, y, width, height;
  private int fps;

  /**
   * Prints a error code from a pixy2 function.
   * 
   * @param errorCode
   */
  private void logPixyError(int errorCode) {
    System.out.println("[PIXY ERROR AGHHHH] " + errorCode);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Initialize Pixy2.
    pixy = Pixy2.createInstance(new SPILink());
    int err = pixy.init(kPixySSPort);
    if (err < 0) {
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
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    // Error codes returned from pixy2 functions.
    int err;

    fps = pixy.getFPS();

    // Get which alliance (true for blue, false for red) the robot is playing on,
    // and set the signature of the pixy accordingly. As of now, signature 1 is for
    // the blue cargo, signature 2 is for the red cargo.
    byte signature = SmartDashboard.getBoolean("Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

    // First argument is set to true to stop the PIXY_RESULT_BUSY error.
    err = pixy.getCCC().getBlocks(true, signature, 25);
    if (err <= 0) {
      blockFound = false;

      if (err < 0)
        logPixyError(err);
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

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}