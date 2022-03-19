package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    public PhotonVision() {
        CameraServer.addServer("http://10.25.30.55:1183/stream.mjpg");
        CameraServer.startAutomaticCapture();
    }
}
