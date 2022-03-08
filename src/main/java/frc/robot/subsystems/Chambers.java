package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.InCANDevice;
import frc.robot.libraries.Deadzone;

public class Chambers extends InCANDevice {
    public static enum BallState {
        IDK,
        Red,
        Green,
        Blue,
        None
    }

    public static BallState[] states = new BallState[4];

    public Chambers(
            int deviceNum) {
        super(deviceNum);
        this.apiID = 32 << 4;
    }

    @Override
    public void onDataReceived(byte[] data) {
        for (int si = 0; si < Math.min(states.length, data.length); ++si)
            states[si] = BallState.values()[data[si]];

        SmartDashboard.putString("Ball states: ", String.format("%s\n", Arrays.toString(states)));
    }

    @Override
    public void periodic() {
        super.periodic(); // Make sure to call super, InCANDevice needs to look for CAN messages
    }
}
