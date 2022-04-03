package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class Battery extends SubsystemBase {

   AHRS ahrs;
   DriveTrain driveTrain;
   XboxController xbox;

   Timer timer = new Timer();

   double minVoltage = getVoltage();
   int batteryPercentage = calculateBatteryPercentage();

   NetworkTableEntry batteryBar = Shuffleboard.getTab("Driver Dashboard")
         .add("Battery", batteryPercentage)
         .withWidget(BuiltInWidgets.kNumberBar).withProperties(
               Map.of("min", 0, "max", 100))
         .withSize(2, 1)
         .withPosition(3, 3)
         .getEntry();
   NetworkTableEntry batteryRuntimeWidget = Shuffleboard.getTab("Technical Info")
         .add("Battery runtime", 0)
         .withSize(2, 1)
         .getEntry();
   NetworkTableEntry minVoltageWidget = Shuffleboard.getTab("Technical Info")
         .add("Minimum voltage ever", 0)
         .withSize(2, 1)
         .getEntry();

   public Battery(AHRS ahrs, DriveTrain driveTrain, XboxController xbox) {
      this.ahrs = ahrs;
      this.driveTrain = driveTrain;
      this.xbox = xbox;
   }

   public double getVoltage() {
      return RobotController.getBatteryVoltage();
   }

   public int calculateBatteryPercentage() {
      return (int) Math.round((getVoltage() - 12.4) / 0.35 * 100);
   }

   public void updateMinBatteryVoltage() {
      minVoltage = Math.min(minVoltage, getVoltage());
      minVoltageWidget.setValue(minVoltage);
      if (minVoltage < Constants.brownOutVoltage) {
         xbox.setRumble(RumbleType.kLeftRumble, 1);
         xbox.setRumble(RumbleType.kRightRumble, 1);
      } else {
         xbox.setRumble(RumbleType.kLeftRumble, 0);
         xbox.setRumble(RumbleType.kRightRumble, 0);
      }
   }

   public void updateBatteryPercentage() {
      batteryPercentage = calculateBatteryPercentage();
      batteryBar.setValue(batteryPercentage);
      // String output = minVoltage > 12.4 ? batteryPercentage + "%"
      // : minVoltage > 12 ? "Avoid high-power functions" : "Replace battery now";
   }

   @Override
   public void periodic() {
      getBatteryRuntime();
      updateMinBatteryVoltage();
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
         batteryRuntimeWidget.setValue(timer.get());
      } else {
         timer.stop();
      }
   }
}
