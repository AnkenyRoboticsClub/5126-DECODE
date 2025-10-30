package org.firstinspires.ftc.teamcode;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

/**
 * Simple UDP listener that expects CSV: "has,tx,ty,ta"
 * Example packets:
 *   "1, 2.3, -1.0, 7.8"
 *   "0, 0.0, 0.0, 0.0"
 *
 * Feed the parsed values into LimelightIO.setMeasurement(...).
 */
public class LimelightUdpClient implements Runnable {
    private final LimelightIO sink;
    private final int port;
    private volatile boolean running = true;

    public LimelightUdpClient(LimelightIO sink, int port) {
        this.sink = sink;
        this.port = port;
    }

    @Override
    public void run() {
        byte[] buf = new byte[256];
        try (DatagramSocket socket = new DatagramSocket(port)) {
            socket.setSoTimeout(200); // non-block forever; loop every ~200ms if no data
            while (running) {
                try {
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);
                    String msg = new String(packet.getData(), 0, packet.getLength()).trim();
                    // Parse CSV
                    String[] p = msg.split(",");
                    if (p.length >= 4) {
                        boolean has = p[0].trim().equals("1");
                        double tx = Double.parseDouble(p[1].trim());
                        double ty = Double.parseDouble(p[2].trim());
                        double ta = Double.parseDouble(p[3].trim());
                        sink.setMeasurement(has, tx, ty, ta);
                    }
                } catch (Exception ignored) { /* timeout or parse fail; just loop */ }
            }
        } catch (Exception ignored) { /* bind failed? make sure port is free */ }
    }

    public void stop() { running = false; }
}
