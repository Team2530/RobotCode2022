package frc.robot.subsystems;

import frc.robot.InCANDevice;

public class CANLights extends InCANDevice {
    public CANLights(int deviceNum) {
        super(deviceNum);
        this.apiID = 25 << 4;
    }

    @Override
    public void periodic() {
    }

    public void runProgram(int prognum) {
        sendData(new byte[] { new Integer(prognum).byteValue() });
    }
}
