package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@Autonomous(name="LL_Align_Then_StopOnWhite", group="Vision")
public class LL_Align_Then_StopOnWhite extends LinearOpMode {

    // ---------------- CHANGE THESE TWO VALUES ----------------
    // Tune once by watching telemetry over gray vs white.
    private static final float  LINE_GAIN        = 2.0f; // 1.0–8.0; increase in dim rooms
    private static final double WHITE_ALPHA_TH   = 1.8;  // <-- SET YOUR THRESHOLD HERE
    // ---------------------------------------------------------

    // motion tuning
    private static final double kP_yaw     = 0.018;  // deg -> power (rotate from tx)
    private static final double kP_x       = 0.030;  // deg -> power (strafe from tx)
    private static final double kP_heading = 0.010;  // deg -> power (IMU heading hold)

    private static final int ROTATE_SIGN = -1;
    private static final int STRAFE_SIGN = +1;

    private static final double maxP = 0.40;         // power cap
    private static final double minP = 0.06;         // minimum effective power

    // deadbands / smoothing
    private static final double deadYaw = 0.8;       // deg (rotation)
    private static final double deadX   = 0.5;       // deg (strafe)
    private static final double alignDoneTolDeg = 1.0; // consider centered when |tx| <= this
    private static final double alpha   = 0.5;       // low-pass on tx
    private static final double slewRate= 1.5;       // power/sec

    // creep toward line
    private static final double DRIVE_FWD_POWER   = 0.18; // forward creep speed
    private static final double STRAFE_CORRECT_CAP= 0.20; // tiny strafe correction cap while creeping

    // hardware
    private DcMotor fl, fr, bl, br;
    private Limelight3A limelight;
    private IMU imu;
    private NormalizedColorSensor lineSensor;

    // state
    private double txFilt = 0.0;
    private double headingSet = 0.0;
    private boolean headingLocked = false;
    private double lastVx=0, lastVy=0, lastOmega=0;
    private long lastSlewMs = 0;

    private enum State { ALIGN_TAG, DRIVE_TO_LINE, LOCKED }
    private State state = State.ALIGN_TAG;

    @Override
    public void runOpMode() {
        // drivetrain
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{fl,fr,bl,br})
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        // IMU (your mounting: Logo LEFT, USB BACKWARD — change if needed)
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        sleep(200);

        // Color sensor (REV Color Sensor V3) named "line"
        lineSensor = hardwareMap.get(NormalizedColorSensor.class, "line");
        lineSensor.setGain(LINE_GAIN);

        headingSet = getHeadingDeg();
        headingLocked = true;
        lastSlewMs = System.currentTimeMillis();

        telemetry.addLine("Align, then stop on white — READY");
        telemetry.addLine(">>> Tune WHITE_ALPHA_TH by watching 'alpha' below");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult r = limelight.getLatestResult();
            boolean valid = r != null && r.isValid();
            double tx = valid ? r.getTx() : 0.0;  // deg
            txFilt = alpha*tx + (1.0-alpha)*txFilt;

            // line sensor read
            NormalizedRGBA c = lineSensor.getNormalizedColors();
            double alphaLight = c.alpha; // brightness proxy
            double rC = c.red, gC = c.green, bC = c.blue;

            double vxCmd = 0, vyCmd = 0, omegaCmd = 0;
            double heading = getHeadingDeg();

            switch (state) {
                case ALIGN_TAG: {
                    if (valid) {
                        // rotate first
                        if (Math.abs(txFilt) > deadYaw) {
                            omegaCmd = clamp(-kP_yaw * txFilt, -maxP, maxP); // tx>0 (tag right) -> CW
                            omegaCmd = withMinPower(omegaCmd, minP);
                            headingLocked = false;
                        } else {
                            // strafe to center
                            if (Math.abs(txFilt) > deadX) {
                                vxCmd = clamp(+kP_x * txFilt, -maxP, maxP); // tx>0 -> strafe right
                                vxCmd = withMinPower(vxCmd, minP);
                            }
                            if (!headingLocked) {
                                headingSet = heading;
                                headingLocked = true;
                            }
                        }
                        // good enough → start creeping to the line
                        if (Math.abs(txFilt) <= alignDoneTolDeg) {
                            state = State.DRIVE_TO_LINE;
                            headingSet = heading;  // lock current heading
                            headingLocked = true;
                        }
                    } else {
                        // search
                        omegaCmd = 0.12;
                        headingLocked = false;
                    }
                } break;

                case DRIVE_TO_LINE: {
                    // keep straight & roughly centered while creeping forward
                    if (valid && Math.abs(txFilt) > deadX) {
                        vxCmd = clamp(+kP_x * txFilt, -STRAFE_CORRECT_CAP, STRAFE_CORRECT_CAP);
                    }
                    vyCmd = DRIVE_FWD_POWER; // creep forward

                    // stop when we hit the white line
                    if (alphaLight >= WHITE_ALPHA_TH) {
                        state = State.LOCKED;
                        vxCmd = 0; vyCmd = 0; omegaCmd = 0;
                    }
                } break;

                case LOCKED: {
                    vxCmd = 0; vyCmd = 0; omegaCmd = 0;
                } break;
            }

            // heading hold (keeps robot straight)
            if (headingLocked) {
                double hErr = angleWrapDeg(headingSet - heading);
                double hold = clamp(kP_heading * hErr, -maxP, maxP);
                omegaCmd += hold;
            }

            // slew limiting
            long now = System.currentTimeMillis();
            double dt = Math.max(1, now - lastSlewMs) / 1000.0;
            vxCmd    = slew(vxCmd,    lastVx,    dt, slewRate);
            vyCmd    = slew(vyCmd,    lastVy,    dt, slewRate);
            omegaCmd = slew(omegaCmd, lastOmega, dt, slewRate);
            lastVx = vxCmd; lastVy = vyCmd; lastOmega = omegaCmd; lastSlewMs = now;

            // mecanum mix (robot-centric)
            double pFL = vyCmd + vxCmd + omegaCmd;
            double pFR = vyCmd - vxCmd - omegaCmd;
            double pBL = vyCmd - vxCmd + omegaCmd;
            double pBR = vyCmd + vxCmd - omegaCmd;
            double max = Math.max(1.0, Math.max(Math.abs(pFL),
                        Math.max(Math.abs(pFR), Math.max(Math.abs(pBL), Math.abs(pBR)))));
            fl.setPower(clamp(pFL/max, -maxP, maxP));
            fr.setPower(clamp(pFR/max, -maxP, maxP));
            bl.setPower(clamp(pBL/max, -maxP, maxP));
            br.setPower(clamp(pBR/max, -maxP, maxP));

            // ---------- TELEMETRY ----------
            telemetry.addLine("=== STATE ===");
            telemetry.addData("state", state);
            telemetry.addData("heading", "%.1f", heading);
            telemetry.addData("headingSet", "%.1f", headingSet);

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("valid", valid);
            telemetry.addData("tx raw", "%.2f", tx);
            telemetry.addData("tx filt", "%.2f", txFilt);
            telemetry.addData("stale(ms)", r!=null ? r.getStaleness() : -1);
            telemetry.addData("pipe", r!=null ? r.getPipelineIndex() : -1);

            telemetry.addLine("=== LINE SENSOR (TUNE HERE) ===");
            telemetry.addData("gain", "%.1f", LINE_GAIN);
            telemetry.addData("alpha (brightness)", "%.2f", alphaLight);
            telemetry.addData("red",   "%.2f", rC);
            telemetry.addData("green", "%.2f", gC);
            telemetry.addData("blue",  "%.2f", bC);
            telemetry.addData("WHITE_ALPHA_TH", "%.2f (change in code)", WHITE_ALPHA_TH);
            telemetry.addLine("Tip: read alpha on GRAY vs WHITE; set threshold between them.");

            telemetry.addLine("=== COMMANDS ===");
            telemetry.addData("vx", "%.2f", vxCmd);
            telemetry.addData("vy", "%.2f", vyCmd);
            telemetry.addData("omega", "%.2f", omegaCmd);
            telemetry.update();
        }

        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }

    // ---------- helpers ----------
    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private static double angleWrapDeg(double a){
        while (a > 180)  a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double withMinPower(double p, double min){
        if (p == 0) return 0;
        double s = Math.signum(p);
        return (Math.abs(p) < min) ? s*min : p;
    }
    private static double slew(double target, double current, double dt, double rate){
        double maxStep = rate * dt;
        double diff = target - current;
        if (diff >  maxStep) return current + maxStep;
        if (diff < -maxStep) return current - maxStep;
        return target;
    }
}
