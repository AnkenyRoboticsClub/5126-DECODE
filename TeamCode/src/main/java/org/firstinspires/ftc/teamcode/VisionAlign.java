package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VisionAlign {

    private final DriveTrain drive;
    private final ImuUtil imu;
    private Limelight3A limelight;

    public VisionAlign(DriveTrain drive, ImuUtil imu) {
        this.drive = drive;
        this.imu = imu;
    }

    public void start(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, Constants.LL_DEVICE_NAME);

        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public LLResult latest() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    // ---------------------------- TeleOp step functions ----------------------------

    public boolean aimStepRobotCentric() {
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        double turn = turnCmd(r.getTx());
        drive.driveRobot(0, 0, turn);
        return Math.abs(r.getTx()) <= Constants.LL_AIM_TOL_DEG;
    }

    public boolean aimAndApproachStepRobotCentric() {
        LLResult r = latest();
        if (r == null || !r.isValid()) {
            drive.stopAll();
            return false;
        }

        double turn = turnCmd(r.getTx());
        double fwd  = forwardCmd(r.getTa());
        drive.driveRobot(0, fwd, turn);
        return onTarget(r.getTx(), r.getTa());
    }

    // ---------------------------- Auto blocking functions ----------------------------

    public boolean aimUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_ALIGN_TIMEOUT_S) {
            if (aimStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return true;
    }

    public boolean aimAndApproachUntil(LinearOpMode op) {
        ElapsedTime t = new ElapsedTime();
        while (op.opModeIsActive() && t.seconds() < Constants.LL_APPROACH_TIMEOUT_S) {
            if (aimAndApproachStepRobotCentric()) break;
            op.idle();
        }
        drive.stopAll();
        return true;
    }

    public int getTagId() {
        LLResult r = latest();
        if (r == null || !r.isValid()) return -1;

        // Get list of fiducial (AprilTag) detections
        java.util.List<FiducialResult> tags = r.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return -1;  // No tags detected
        }

        // Return ID of the first (main) tag
        return tags.get(0).getFiducialId();
    }

    public String motifFromTag(int id) {
        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return "UNKNOWN";
        }
    }

    public String scanMotif() {
        int id = getTagId();
        return motifFromTag(id);
    }

    // ---------------------------- Math Helpers ----------------------------

    private static double turnCmd(double tx) {
        if (Math.abs(tx) <= Constants.LL_AIM_TOL_DEG) return 0;

        double u = Constants.LL_K_TURN * tx;
        u += Math.signum(u) * Constants.LL_MIN_TURN;
        return clamp(u, -Constants.LL_MAX_TURN, Constants.LL_MAX_TURN);
    }

    private static double forwardCmd(double ta) {
        double err = Constants.LL_TARGET_AREA - ta;
        if (Math.abs(err) <= Constants.LL_APPROACH_TOL_TA) return 0;

        double u = Constants.LL_K_FORWARD * err;
        u += Math.signum(u) * Constants.LL_MIN_FORWARD;
        return clamp(u, -Constants.LL_MAX_FORWARD, Constants.LL_MAX_FORWARD);
    }

    private static boolean onTarget(double tx, double ta) {
        return Math.abs(tx) <= Constants.LL_AIM_TOL_DEG &&
                ta >= Constants.LL_TARGET_AREA - Constants.LL_APPROACH_TOL_TA;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
