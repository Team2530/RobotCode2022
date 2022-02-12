// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base for InCANceivable devices -- intended to be overriden and used as a
 * framework for subsystems handled by Arduinos on the CAN bus.
 */
public abstract class InCANDevice extends SubsystemBase {
    // Identifiers unique to the CAN handling on the Arduinos
    private static final int CAN_MFG = 8;
    private static final int CAN_TYPE = 10;

    protected int deviceNum, apiID;
    protected CAN conn;

    protected CANData lastReceivedData = new CANData();

    public InCANDevice(int deviceNum) {
        this.deviceNum = deviceNum;
        conn = new CAN(deviceNum, CAN_MFG, CAN_TYPE);
    }

    // Override for custom data handling
    public void onDataReceived(byte[] data) {

    }

    @Override
    public void periodic() {
        if (conn.readPacketNew(apiID, lastReceivedData)) {
            onDataReceived(Arrays.copyOf(lastReceivedData.data, lastReceivedData.length));
        }
    }

    public void sendData(byte[] data) {
        conn.writePacket(data, apiID);
    }
}