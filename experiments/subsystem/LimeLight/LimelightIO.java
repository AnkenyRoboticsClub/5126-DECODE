package org.firstinspires.ftc.teamcode;

/** Data sink for Limelight measurements (transport-agnostic). */
public interface LimelightIO {
    /**
     * Push one measurement into the vision pipeline.
     * @param hasTarget true if a valid AprilTag/target is detected
     * @param txDeg     horizontal offset from crosshair, degrees (+right)
     * @param tyDeg     vertical offset (unused here, but kept for completeness)
     * @param ta        target area (0..100-ish depending on LL settings)
     */
    void setMeasurement(boolean hasTarget, double txDeg, double tyDeg, double ta);
}
