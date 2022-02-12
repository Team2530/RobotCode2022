package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.InCANDevice;
import frc.robot.libraries.Deadzone;

// public class InCANceivable extends SubsystemBase {
// private int dvc_num;

// // TODO: Just put this where needed, make the multiplexer a small static
// class
// // with switchOn, switchOff, and switchTo methods.
// ColorSensorV3 cs = new ColorSensorV3(Port.kOnboard); // Port 0x52 on I2C,
// hooked to multiplexer channel 0
// // I2C multiplexer = new I2C(Port.kOnboard, 0x70);

// private static final int MFG = 8; // Fill in from InCANCievable github
// private static final int TYPE = 10; // Fill in from InCANCievable github

// CAN conn;
// CANData recv = new CANData();

// public InCANceivable(int can_id) {
// conn = new CAN(can_id, MFG, TYPE);
// }

// @Override
// public void periodic() {
// // ...Periodically called whenever WPILib feels like it :)
// // System.out.println(recv.length);
// if (conn.readPacketNew(25, recv)) {
// // TODO: Callbacks?

// // Received data
// String s = "";
// for (int i = 0; i < recv.length; i++)
// s += ", " + Byte.toString(recv.data[i]);
// s = s.substring(2);
// System.out.printf("Received %d bytes: [%s]\n", recv.length, s);
// }

// // Switch the numtiplexer port (register 0) with the shift amount
// // multiplexer.write(0, 1 << 0);

// Integer red = (int) (cs.getColor().red * 255 * 1.6);
// Integer green = (int) (cs.getColor().green * 255 * 1.4);
// Integer blue = (int) (cs.getColor().blue * 255 * 1.6);
// System.out.printf("(%d, %d, %d)\n", red, green, blue);

// // Send color to Arduino
// // NOTE: this will eventually be handled by the arduino itself, and we will
// have
// // to receive color from the arduino over CAN.
// conn.writePacket(new byte[] {
// red.byteValue(),
// green.byteValue(),
// blue.byteValue()
// }, 25);
// }

// // @Override
// // public void simulationPeriodic() {
// // // This method will be called once per scheduler run during simulation
// // }

// public void runProgram(int prog) {
// conn.writePacket(new byte[] { (byte) prog }, 25);
// // conn.writePacketRepeating(new byte[] { (byte) prog }, 25, 10);
// }

// public void stopProgram() {
// // conn.stopPacketRepeating(25);
// conn.writePacket(new byte[0], 25);
// }
// }

public class BallDetection extends InCANDevice {
    public static enum BallState {
        IDK,
        Red,
        Green,
        Blue,
        None
    }

    public static BallState[] states = new BallState[2];

    public BallDetection(
            int deviceNum) {
        super(deviceNum);
        this.apiID = 32 << 4;
    }

    @Override
    public void onDataReceived(byte[] data) {
        for (int si = 0; si < 2; ++si)
            states[si] = BallState.values()[data[si]];

        SmartDashboard.putString("Ball states: ", String.format("%s\n", Arrays.toString(states)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lerp test", Deadzone.deadZone(1.0, 0.1));
        super.periodic(); // Make sure to call super, InCANDevice needs to look for CAN messages
    }
}
