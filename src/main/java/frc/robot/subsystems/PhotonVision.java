package frc.robot.subsystems;

import java.awt.Color;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class PhotonVision extends SubsystemBase {
    private int camera;
    private double yawTarget = 0.0;
    PhotonCamera BSideCamera = new PhotonCamera("Breaker");
    PhotonCamera ESideCamera = new PhotonCamera("Ethernet");
    PhotonCamera curCamera = BSideCamera;
    private final double yawRate = 310.0;
    double forwardSpeed;
    double gain = 1.0;
    double rotationSpeed;
    double kP = 0.5, kI = 0, kD = 0;
    private int pipelineIndex = 1;
    private double degrees = -0.3;
    PIDController forwardController = new PIDController(kP, kI, kD);
    PIDController turnController = new PIDController(kP, kI, kD);
    private String camerastring;
    private Joystick stick;
    private DriveTrain driveTrain;
    private Indicators indicators;

    @Override
    public void periodic() {
        // indicators.setIndicatorColor(0, PhotonUtils != null /
        // Color.GREEN : Color.RED);
    }

    public PhotonVision(Joystick stick, DriveTrain driveTrain, Indicators indicators) {
        this.stick = stick;
        this.driveTrain = driveTrain;
        CameraServer.addServer("http://10.25.30.55:1183/stream.mjpg");
        CameraServer.startAutomaticCapture();
    }

    public void setPipelineFromAlliance() {
        pipelineIndex = DriverStation.getAlliance() == Alliance.Blue ? 2 : 1;
        BSideCamera.setPipelineIndex(pipelineIndex);
        ESideCamera.setPipelineIndex(pipelineIndex);
    }

    public void invertCams() {
        camera = camera * -1;
    }

    public void drive() {
        if (camera == 1) {
            curCamera = BSideCamera;
        } else {
            curCamera = ESideCamera;
        }
        PhotonPipelineResult result = curCamera.getLatestResult();

        System.out.println(result.hasTargets());
        if (result.hasTargets()) {
            System.out.println();
            // gain = Math.min(0.75, Math.sqrt(result.getBestTarget().getArea()) / 2);
            rotationSpeed = turnController.calculate(result.getBestTarget().getYaw()
                    / 30, -3);
            if (result.getBestTarget().getArea() > 7.5)
                rotationSpeed = 0.0;
            forwardSpeed = camera * stick.getY() / 2;
        } else {
            System.out.println("No Ball(s)");
            rotationSpeed = 0;
        }

        driveTrain.singleJoystickDrive(
                0,
                forwardSpeed,
                rotationSpeed);
    }
}
