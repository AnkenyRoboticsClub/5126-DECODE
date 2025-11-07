package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LimelightResults;
import com.qualcomm.hardware.limelightvision.LimelightVisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * VisionAlign: high-level helper to align to an AprilTag using Limelight 4A (USB-C).
 * - Reads tx/ta via FTC's LimelightVisionPortal
 * - Computes turn/forward commands from Constants tunables
 * - Offers TeleOp "step" methods (non-blocking) and Auto "until" methods (blocking)
 *
 * Requires DriveTrain to have either:
 *   - driveRobot(x, y, rx)  // robot-centric (recommended for vision), or
 *   - use driveFieldCentric(...) inline (see comments below).
 */
public class VisionAlign {

    private final DriveTrain drive;
    private final ImuUtil imu;

    private Limelight3A limelight;
    private LimelightVisionPortal portal;

    public VisionAlign(DriveTrain drive, ImuUtil imu) {
        this.drive = drive;
        this.imu   = imu;
    }

    /** Call once during init. Assumes the device name in RC config matches Constants.LL_DEVICE_NAME. */
    public void start(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, Constants.LL_DEVICE_NAME);
        portal = LimelightVisionPortal.easyCreateWithDefaults(limelight);
        // If you have multiple pipelines, you can set it here:
        // portal.setPipelineIndex(0); // (AprilTag pipeline)
        // LED control is pipeline-controlled by default.
    }

    /** Optional: stop/cleanup on OpMode end. */
    public void stop() {
        if (portal != null) {
            // No hard requirement to close, but you can stop streaming if desired:
            // portal.stopStreaming();
        }
    }

    // -------------------- TeleOp: one-cycle steps (NON-BLOCKING) --------------------

    /** Turn to center the tag (rx only). Returns true when |tx| <= aim tolerance. */
    public boolean aimStepRobotCentric() {
        LimelightResults r = portal.getLatestResults();
        if (r == null || !r.isValid()) { drive.stopAll(); return false; }

        double turn = turnCmd(r.getTx());
        // Robot-centric: only rotate this cycle.
        drive.driveRobot(0, 0, turn);
        return Math.abs(r.getTx()) <= Constants.LL_AIM_TOL_DEG;
    }

    /** Turn + creep forward toward standoff. Returns true when on target (aimed & close). */
    public boolean aimAndApproachStepRobotCentric() {
        LimelightResults r = portal.getLatestResults();
        if (r == null || !r.isValid()) { drive.stopAll(); return false; }

        double turn = turnCmd(r.getTx());
        double fwd  = forwardCmd(r.getTa());
        drive.driveRobot(0, fwd, turn);
        return onTarget(r.getTx(), r.getTa());
    }

    // If you prefer field-centric, replace driveRobot with:
    //   double heading = imu.getHeadingRad();
    //   drive.driveFieldCentric(0, /*y*/ fwd, /*rx*/ turn, heading, false, false);

    // -------------------- Auto: blocking helpers with timeouts ----------------------

    public boolean aimUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_ALIGN_TIMEOUT_S) {
            if (aimStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        LimelightResults r = portal.getLatestResults();
        return (r != null && r.isValid() && Math.abs(r.getTx()) <= Constants.LL_AIM_TOL_DEG);
    }

    public boolean aimAndApproachUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_APPROACH_TIMEOUT_S) {
            if (aimAndApproachStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        LimelightResults r = portal.getLatestResults();
        return (r != null && r.isValid() && onTarget(r.getTx(), r.getTa()));
    }

    // -------------------- Math helpers (use Constants tunables) --------------------

    private static double turnCmd(double txDeg) {
        double err = txDeg;
        if (Math.abs(err) <= Constants.LL_AIM_TOL_DEG) return 0;
        double u = Constants.LL_K_TURN * err;
        u += Math.signum(u) * Constants.LL_MIN_TURN; // push through static friction
        return clamp(u, -Constants.LL_MAX_TURN, Constants.LL_MAX_TURN);
    }

    private static double forwardCmd(double ta) {
        double err = Constants.LL_TARGET_AREA - ta; // positive => too far, drive forward
        if (Math.abs(err) <= Constants.LL_APPROACH_TOL_TA) return 0;
        double u = Constants.LL_K_FORWARD * err;
        u += Math.signum(u) * Constants.LL_MIN_FORWARD;
        return clamp(u, -Constants.LL_MAX_FORWARD, Constants.LL_MAX_FORWARD);
    }

    private static boolean onTarget(double txDeg, double ta) {
        boolean aimed  = Math.abs(txDeg) <= Constants.LL_AIM_TOL_DEG;
        boolean close  = ta >= (Constants.LL_TARGET_AREA - Constants.LL_APPROACH_TOL_TA);
        return aimed && close;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
