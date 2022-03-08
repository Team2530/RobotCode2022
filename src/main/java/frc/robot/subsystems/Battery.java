package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;

public class Battery extends SubsystemBase{
   public void displayBatteryVoltage() {
      double power = RobotController.getBatteryVoltage();
      SmartDashboard.putNumber("Battery voltage", power);
   }
   @Override
   public void periodic() {
     displayBatteryVoltage();
  
}
}
