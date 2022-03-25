package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;

public class Battery extends SubsystemBase {

   AHRS ahrs;
   XboxController xbox = new XboxController(Constants.xboxport);

   public Battery(AHRS ahrs, XboxController xbox) {
      this.ahrs = ahrs;
      this.xbox = xbox;
   }

   double minVoltage = getVoltage();
   int batteryPercentage = calculateBatteryPercentage();

   public double getVoltage() {
      return RobotController.getBatteryVoltage();
   }

   public int calculateBatteryPercentage() {
      return (int) Math.round((getVoltage() - 12.4) / 0.35 * 100);
   }

   public void updateMinBatteryVoltage() {
      minVoltage = Math.min(minVoltage, getVoltage());
      SmartDashboard.putNumber("Minimum voltage ever", minVoltage);
      if (minVoltage < Constants.brownOutVoltage) {
         xbox.setRumble(RumbleType.kLeftRumble, 1);
         xbox.setRumble(RumbleType.kRightRumble, 1);
      }
   }

   public void updateBatteryPercentage() {
      batteryPercentage = calculateBatteryPercentage();
      String output = minVoltage > 12.4 ? batteryPercentage + "%"
            : minVoltage > 12 ? "Avoid high-power functions" : "Replace battery now";
      SmartDashboard.putString("Battery", output);
   }

   @Override
   public void periodic() {
      updateMinBatteryVoltage();
      if (!ahrs.isMoving() && !ahrs.isRotating())
         updateBatteryPercentage();
   }
}
