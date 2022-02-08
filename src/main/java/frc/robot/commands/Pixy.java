package frc.robot.commands;

import frc.robot.subsystems.*;

import java.io.Console;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {
  private final Pixy2 pixy;

  // ints for ball data
  Block loadedblock;
  double ballwidth;
  double ballheight;
  double ballx;
  double ballY;
  double ballAngle;
  double ballAge;
  double lastBlockCount;

  public Pixy() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    loadedblock = this.getBiggestBlock();
  }

  public Block getBiggestBlock() {
    // Finds blocks that match signature 1,
    // finds the largest of them,
    // then returns it in the original data-retrievable format

    double blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25 /* limit on blocks to return */);
    if (blockCount < 0) {
      blockCount = lastBlockCount;
    }
    lastBlockCount = blockCount;
    SmartDashboard.putNumber("block count", blockCount);

    if (blockCount <= 0) {
      return null;
    }
    ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
    // blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the
    // Pixy2

    Block largestBlock = null;
    for (Block block : blocks) { // Loops through all blocks and finds the widest one
      if (block.getWidth() > largestBlock.getWidth() || block.getHeight() > largestBlock.getHeight()) {
        largestBlock = block;
      }
    }
    return largestBlock;
  }

  @Override
  public void periodic() {

    // Gets Pixy data and stores in variables
    loadedblock = this.getBiggestBlock();
    System.out.println(loadedblock);
    if (loadedblock != null) {
      ballwidth = loadedblock.getWidth();
      ballheight = loadedblock.getHeight();
      ballx = loadedblock.getX(); // location on screen for ... maybe own angle calculations
      ballY = loadedblock.getY();
      ballAngle = loadedblock.getAngle(); // built in angle returning function
      System.out.println("ballwidth : " + ballwidth + "; ballHeight : " + ballheight + "; ballx : " + ballx
          + "; bally : " + ballY + "; ballangle : " + ballAngle);
    }
  }

  public double getX() {
    return ballx;
  }

  public double getHeight() {
    return ballwidth;
  }

  public double getWidth() {
    return ballheight;
  }

  public double getArea() {
    return ballwidth * ballheight;
  }
}