package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;

public class MT6835 {
    private CAN canDevice;
    private CANData canData;
    private final int deviceID;
    private final boolean debugMode;

    public MT6835(int deviceID, boolean debugMode) {
        this.deviceID = deviceID;
        this.debugMode = debugMode;
        this.canDevice = new CAN(deviceID, 8, 10);
        this.canData = new CANData();
    }

    /**
     * Reads the latest packet from the CAN device and returns the angle as a normalized value.
     *
     * @param apiID The API ID to use for reading.
     * @return A double representing the angle between 0 and 1, or -1 if no valid packet is received.
     */
    public double readAngle(int apiID) {
        if (canDevice.readPacketLatest(apiID, canData)) {
            byte[] receivedData = canData.data;

            if (canData.length >= 3) {
                int counts = ((receivedData[0] & 0xFF) << 16) |
                             ((receivedData[1] & 0xFF) << 8) |
                             (receivedData[2] & 0xFF);

                double angle = counts / (double) (1 << 21); // Normalize based on 2^21 (max value for 21 bits)

                if (debugMode) {
                    System.out.printf("Device %d: Read angle = %.6f (API_ID=%d, Data=%s)%n",
                            deviceID, angle, apiID, bytesToHex(receivedData));
                }

                return angle;
            } else {
                if (debugMode) {
                    System.out.println("Device " + deviceID + ": Received CAN Packet with insufficient data.");
                }
            }
        } else {
            if (debugMode) {
                System.out.println("Device " + deviceID + ": No CAN message received.");
            }
        }
        return -1; // Return -1 to indicate an error
    }

    /**
     * Sends a CAN packet to the device.
     *
     * @param apiID The API ID for the packet.
     * @param data  The payload to send.
     */
    public void sendPacket(int apiID, byte[] data) {
        try {
            canDevice.writePacket(data, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent CAN Packet: API_ID=%d, Data=%s%n",
                        deviceID, apiID, bytesToHex(data));
            }
        } catch (Exception e) {
            if (debugMode) {
                System.err.printf("Device %d: Failed to send CAN Packet: %s%n", deviceID, e.getMessage());
            }
        }
    }

    /**
     * Helper function to convert a byte array to a hexadecimal string.
     *
     * @param bytes The byte array to convert.
     * @return A string representation of the byte array in hexadecimal format.
     */
    private String bytesToHex(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return sb.toString().trim();
    }
}
