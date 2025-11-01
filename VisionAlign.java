package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * VisionAlign wraps the Limelight helper and DriveTrain so you can:
 *  - run an "aim" step (rx only) or "aim+approach" step (rx + forward)
 *  - use it in TeleOp loops (non-blocking, one step per loop)
 *  - use it in Auto (blocking helpers with timeouts)
 *
 * NOTE: By default we drive ROBOT-CENTRIC for vision, so turn and forward map directly.
 * If you prefer field-centric, see the comment where we call DriveTrain.
 */
public class VisionAlign {
    private final DriveTrain drive;
    private final ImuUtil imu;
    private final LimelightAprilTagHelper ll;
    private Thread udpThread;
    private LimelightUdpClient udpClient;

    public VisionAlign(DriveTrain drive, ImuUtil imu) {
        this.drive = drive;
        this.imu = imu;

        // Build helper from Constants
        LimelightAprilTagHelper.Params params = new LimelightAprilTagHelper.Params(
                Constants.LL_K_TURN, Constants.LL_MAX_TURN, Constants.LL_MIN_TURN, Constants.LL_AIM_TOL_DEG,
                Constants.LL_K_FORWARD, Constants.LL_MAX_FORWARD, Constants.LL_MIN_FORWARD,
                Constants.LL_TARGET_AREA, Constants.LL_APPROACH_TOL_TA
        );
        this.ll = new LimelightAprilTagHelper(params);
    }

    /** Start UDP reader (call once in init). Do NOT call from the loop every cycle. */
    public void startUdp() {
        stopUdp(); // safety
        udpClient = new LimelightUdpClient(ll, Constants.LL_UDP_PORT);
        udpThread = new Thread(udpClient);
        udpThread.start();
    }

    /** Stop UDP reader (call on OpMode stop). */
    public void stopUdp() {
        if (udpClient != null) udpClient.stop();
        udpClient = null;
        udpThread = null;
    }

    public LimelightAprilTagHelper getHelper() { return ll; }

    // ---------------- TeleOp-style step methods (NON-BLOCKING) ----------------

    /**
     * One cycle of "aim only".
     * Call this once per loop WHILE the driver is holding the aim button.
     * Returns true if we're inside the aim tolerance.
     */
    public boolean aimStepRobotCentric() {
        if (!ll.hasTarget()) {
            // No target: do nothing (or optionally stop drive)
            drive.stopAll();
            return false;
        }
        double turn = ll.turnCmd(); // rx
        // Robot-centric: (x=0,y=0) and only apply rotation to face the tag.
        drive.driveRobot(0, 0, turn);
        return Math.abs(ll.getTxDeg()) <= Constants.LL_AIM_TOL_DEG;
    }

    /**
     * One cycle of "aim + approach".
     * Call this once per loop WHILE the driver is holding the vision-assist button.
     * Robot-centric: turn = rx from tx; forward = y from area error.
     */
    public boolean aimAndApproachStepRobotCentric() {
        if (!ll.hasTarget()) {
            drive.stopAll();
            return false;
        }
        double turn = ll.turnCmd();
        double fwd  = ll.forwardCmd();
        drive.driveRobot(0, fwd, turn);
        return ll.onTarget();
    }

    // ---------------- Auto-style helpers (BLOCKING with timeouts) ----------------

    /** Block until aimed or timeout. Returns true if aimed within tolerance. */
    public boolean aimUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_ALIGN_TIMEOUT_S) {
            if (aimStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return Math.abs(ll.getTxDeg()) <= Constants.LL_AIM_TOL_DEG;
    }

    /** Block until aimed & at standoff (area reached) or timeout. */
    public boolean aimAndApproachUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_APPROACH_TIMEOUT_S) {
            if (aimAndApproachStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return ll.onTarget();
    }
}
