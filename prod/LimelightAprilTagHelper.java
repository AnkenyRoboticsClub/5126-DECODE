package org.firstinspires.ftc.teamcode;

/**
 * Limelight AprilTag helper.
 * - Holds latest vision data (has, tx, ty, ta)
 * - Computes suggested turn/forward commands using simple P control with small feed-forward
 * - Provides a boolean "onTarget" to indicate "aimed and at standoff"
 *
 * This class implements LimelightIO so your UDP/HTTP client can push frames here directly.
 */
public class LimelightAprilTagHelper implements LimelightIO {
    private volatile boolean hasTarget;
    private volatile double txDeg;
    private volatile double tyDeg;
    private volatile double ta;

    // Tunables (injected from Constants so you can tweak from a single place)
    public static class Params {
        public double kTurn, maxTurn, minTurn, aimTolDeg;
        public double kForward, maxForward, minForward, targetArea, approachTolTA;

        public Params(double kTurn, double maxTurn, double minTurn, double aimTolDeg,
                      double kForward, double maxForward, double minForward,
                      double targetArea, double approachTolTA) {
            this.kTurn = kTurn; this.maxTurn = maxTurn; this.minTurn = minTurn; this.aimTolDeg = aimTolDeg;
            this.kForward = kForward; this.maxForward = maxForward; this.minForward = minForward;
            this.targetArea = targetArea; this.approachTolTA = approachTolTA;
        }
    }

    private final Params p;

    public LimelightAprilTagHelper(Params params) {
        this.p = params;
    }

    // Transport pushes data here
    @Override
    public void setMeasurement(boolean hasTarget, double txDeg, double tyDeg, double ta) {
        this.hasTarget = hasTarget;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.ta = ta;
    }

    // Accessors (for telemetry, debug)
    public boolean hasTarget() { return hasTarget; }
    public double getTxDeg()   { return txDeg; }
    public double getTyDeg()   { return tyDeg; }
    public double getTa()      { return ta; }

    /** Turn command to center the tag horizontally (use as rx). */
    public double turnCmd() {
        if (!hasTarget) return 0;
        double err = txDeg;                    // +right => turn right
        if (Math.abs(err) <= p.aimTolDeg) return 0;
        double u = p.kTurn * err;
        // Small feed-forward to push through static friction when we do need to turn
        u += Math.signum(u) * p.minTurn;
        return clamp(u, -p.maxTurn, p.maxTurn);
    }

    /** Forward command to reach desired area (use as y for robot-centric drive). */
    public double forwardCmd() {
        if (!hasTarget) return 0;
        double err = p.targetArea - ta;        // positive => we are too far; drive forward
        if (Math.abs(err) <= p.approachTolTA) return 0;
        double u = p.kForward * err;
        u += Math.signum(u) * p.minForward;
        return clamp(u, -p.maxForward, p.maxForward);
    }

    /** True when we are aimed and near the desired "ta" (standoff distance). */
    public boolean onTarget() {
        return hasTarget && Math.abs(txDeg) <= p.aimTolDeg && ta >= (p.targetArea - p.approachTolTA);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
