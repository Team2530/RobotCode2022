package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.InCANDevice;

public class Indicators extends InCANDevice {
    static final int NUM_INDICATORS = 2;
    static final int STRIDE = 3;
    static final long SEND_DELAY_MILLIS = 100; // 100ms beteen updates

    private byte[] buffer = new byte[NUM_INDICATORS * STRIDE]; // Must be <8
    private long lastSentTS = System.currentTimeMillis();

    public Indicators(int deviceNum) {
        super(deviceNum);
        this.apiID = 25 << 4;
    }

    @Override
    public void periodic() {
        if (System.currentTimeMillis() - lastSentTS >= SEND_DELAY_MILLIS) {
            lastSentTS = System.currentTimeMillis();
            updateIndicators();
        }
    }

    public void setIndicatorColor(int indicator, Color c) {
        setIndicatorColor(indicator,
                (byte) (c.red * 255.99),
                (byte) (c.green * 255.99),
                (byte) (c.blue * 255.99));
    }

    public void setIndicatorColor(int indicator, byte r, byte g, byte b) {
        if (indicator >= 3 || indicator < 0)
            System.err.println("Indicators::setIndicatorColor() ERROR: indicator index out of range.");
        buffer[indicator * STRIDE + 0] = r;
        buffer[indicator * STRIDE + 1] = g;
        buffer[indicator * STRIDE + 2] = b;
    }

    private void updateIndicators() {
        sendData(buffer);
    }
}
