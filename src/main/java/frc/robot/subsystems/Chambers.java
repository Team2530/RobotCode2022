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

    /** An array of the contents of each ball chamber. */
    public static BallState[] states = new BallState[4];
    /**
     * An array of whether or not each ball chamber has a detected ball. `true`
     * indicates a ball's presence, `false` indicates that there is no ball or IDK.
     * Do not assume that `false` means that there is no ball, because it could be
     * IDK -- use ballNotDetected instead.
     */
    public static boolean[] ballDetected = new boolean[states.length];
    /**
     * An array of whether or not each ball chamber has a detected absence of a
     * ball. `true` indicates a ball's absence, `false` indicates that there is no
     * detected absence or IDK. Do not assume that `false` indicates the existence
     * of a ball, because it could be IDK -- use ballDetected instead.
     */
    public static boolean[] ballNotDetected = new boolean[states.length];

    public Chambers(
            int deviceNum) {
        super(deviceNum);
        this.apiID = 32 << 4;
    }

    @Override
    public void onDataReceived(byte[] data) {
        for (int si = 0; si < Math.min(states.length, data.length); ++si) {
            states[si] = BallState.values()[data[si]];
            ballDetected[si] = states[si] == BallState.Red || states[si] == BallState.Blue;
            ballNotDetected[si] = states[si] == BallState.None || states[si] == BallState.Green;
        }

        SmartDashboard.putString("Ball states: ", String.format("%s\n", Arrays.toString(states)));
    }

    @Override
    public void periodic() {
        super.periodic(); // Make sure to call super, InCANDevice needs to look for CAN messages
    }
}
