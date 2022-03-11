package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrain;

public class Battery extends SubsystemBase {

   AHRS ahrs;

   public Battery(AHRS ahrs) {
      this.ahrs = ahrs;
   }

   double minVoltage = getVoltage();
   int batteryPercentage = calculateBatteryPercentage();

   public double getVoltage() {
      return RobotController.getBatteryVoltage();
   }

   public int calculateBatteryPercentage() {
      return (int) Math.round((getVoltage() - 12) / 0.75 * 100);
   }

   public void updateMinBatteryVoltage() {
      minVoltage = Math.min(minVoltage, getVoltage());
      SmartDashboard.putNumber("Minimum voltage ever", minVoltage);
   }

   public void updateBatteryPercentage() {
      batteryPercentage = calculateBatteryPercentage();
      SmartDashboard.putString("Battery", batteryPercentage + "%");
   }


   @Override
   public void periodic() {
      updateMinBatteryVoltage();
      if (!ahrs.isMoving() && !ahrs.isRotating()) updateBatteryPercentage();
   }
}
