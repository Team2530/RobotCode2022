package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


//TODO: define port for shooter 
public class Shooter extends SubsystemBase{
    private static WPI_TalonFX[] shooterMotor = {
        new WPI_TalonFX(Constants.UPPER_INTAKE_PORT) 
    };
    
}
