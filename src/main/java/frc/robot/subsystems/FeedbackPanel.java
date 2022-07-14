package frc.robot.subsystems;

import frc.robot.InCANDevice;

public class FeedbackPanel extends InCANDevice {
    public static enum PanelMode {
        Status(0),
        Boot(1),
        Climb(2);

        private final byte value;

        PanelMode(final int v) {
            value = new Integer(v).byteValue();
        }

        public byte getValue() {
            return value;
        }
    };

    public FeedbackPanel(int deviceNum) {
        super(deviceNum);
        this.apiID = 28 << 4;
        setDisplayMode(PanelMode.Boot);
    }

    @Override
    public void periodic() {
    }

    public void setDisplayMode(PanelMode mode) {
        sendData(new byte[] { mode.getValue() });
    }
}
