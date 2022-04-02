package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class Battery extends SubsystemBase {

   AHRS ahrs;
   DriveTrain driveTrain;
   XboxController xbox;

   Timer timer = new Timer();

   public Battery(AHRS ahrs, DriveTrain driveTrain, XboxController xbox) {
      this.ahrs = ahrs;
      this.driveTrain = driveTrain;
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
      getBatteryRuntime();
      updateMinBatteryVoltage();
      if (!ahrs.isMoving() && !ahrs.isRotating())
         updateBatteryPercentage();
   }

   public void getBatteryRuntime() {
      double a = Math.abs(driveTrain.motorBL.getMotorOutputVoltage());
      double b = Math.abs(driveTrain.motorBR.getMotorOutputVoltage());
      double q = Math.abs(driveTrain.motorFL.getMotorOutputVoltage());
      double e = Math.abs(driveTrain.motorFR.getMotorOutputVoltage());

      if ((a > .1) || (b > .1) || (q > .1) || (e > .1) || xbox.getRawButton(3) || xbox.getRawButton(1)
            || xbox.getRawButton(2)) {
         timer.start();
         SmartDashboard.putNumber("Battery Runtime", timer.get());
      } else {
         timer.stop();
      }
   }
}
