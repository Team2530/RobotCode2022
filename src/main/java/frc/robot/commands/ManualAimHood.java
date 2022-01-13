/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Revolver;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualAimHood extends CommandBase {

  Joystick stick;
  Hood hood;
  Revolver revolver;

  public ManualAimHood(Joystick stick, Hood hood, Revolver revolver) {
    this.stick = stick;
    this.hood = hood;
    this.revolver = revolver;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stick.getRawButton(7)) {
      // hood.flywheelSpeedSetPercentOutput(1);
      // hood.setHood(0.55);
    } else if (stick.getRawButton(8)) {
      // hood.flywheelSpeedSetPercentOutput(0.8);
      // hood.setHood(0.55);
    } else if (stick.getRawButton(9)) {
      // hood.flywheelSpeedSetPercentOutput(0.8);
      // hood.setHood(0.35);
    } else if (stick.getRawButton(10)) {
      // hood.flywheelSpeedSetPercentOutput(0.6);
      // hood.setHood(0.35);
    } else {
      // hood.flywheelSpeedSetPercentOutput(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hood.flywheelRotateSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
