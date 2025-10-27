package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="GREEN BOT", group="Linear OpMode")
public class GREENBOT extends LinearOpMode {

    // ======= Drive tuning (your originals) =======
    static final double TURN_SPEED = 1.0;
    static final double MOVING_SPEED_SLOW = 0.4;
    static final double MOVING_SPEED = 0.9;
    static final double MOVING_SPEED_FAST = 1.0;
    private static final double SCOOTCH_POWER = 0.3;
    private static final long   SCOOTCH_DURATION_MS = 200;

    // ======= Flywheel tuning (easy to read / adjust) =======
    // Adjust to your motor/gearbox ticks per rev. Common goBILDA 312RPM is ~537.7
    private static final double TICKS_PER_REV   = 537.7;
    private static final double MAX_TEST_RPM    = 2800;   // top of your usable range for testing
    private static final double RPM_TOL         = 50;     // +/- tolerance for "at speed"
    // Optional feeder positions (only used if you wire "feeder" servo)
    private static final double FEED_POS = 0.80;
    private static final double REST_POS = 0.20;

    // ======= Small, readable helpers =======
    /** 1D distance(ft) -> RPM map with linear interpolation. Keep arrays sorted. */
    static class DistanceRpmMap {
        // Start with guesses; tune these on-field and update.
        private static final double[] DIST_FT = { 2.0, 3.0, 4.0, 5.0, 6.0 };
        private static final double[] RPM     = { 1600, 1800, 2000, 2300, 2600 };

        static double rpmFromFt(double ft) {
            if (ft <= DIST_FT[0]) return RPM[0];
            int last = DIST_FT.length - 1;
            if (ft >= DIST_FT[last]) return RPM[last];

            int i = 0;
            while (i < last && ft > DIST_FT[i+1]) i++;
            double x0 = DIST_FT[i],   x1 = DIST_FT[i+1];
            double y0 = RPM[i],       y1 = RPM[i+1];
            double t = (ft - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    /** Tiny wrapper around DcMotorEx for velocity control, keeps logic tidy. */
    static class FlywheelController {
        private final DcMotorEx motor;
        private final double    ticksPerRev;

        FlywheelController(DcMotorEx motor, double ticksPerRev) {
            this.motor = motor;
            this.ticksPerRev = ticksPerRev;

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        private double rpmToTps(double rpm) { return (rpm / 60.0) * ticksPerRev; }
        private double tpsToRpm(double tps) { return (tps / ticksPerRev) * 60.0; }

        void setTargetRpm(double rpm) {
            motor.setVelocity(rpmToTps(rpm));
        }

        double getMeasuredRpm() {
            return tpsToRpm(motor.getVelocity());
        }
    }

    // ======= Hardware =======
    // Drive
    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    // Flywheel (DcMotorEx for velocity). Keep your config name "motor4".
    private DcMotorEx motorFlyWheel;
    private FlywheelController fly;

    // Optional feeder servo (null-safe)
    private Servo feeder;

    // ======= “Fake distance” testing at home =======
    private double testDistanceFt = 3.0;   // start here; tweak with dpad
    private boolean dpadUpWas, dpadDownWas;

    @Override
    public void runOpMode() throws InterruptedException {
        // ----- Hardware map -----
        motorFrontLeft  = hardwareMap.dcMotor.get("motor0");
        motorBackLeft   = hardwareMap.dcMotor.get("motor2");
        motorFrontRight = hardwareMap.dcMotor.get("motor1");
        motorBackRight  = hardwareMap.dcMotor.get("motor3");

        motorFlyWheel = hardwareMap.get(DcMotorEx.class, "motor4");

        // Optional feeder (safe if not present)
        try {
            feeder = hardwareMap.get(Servo.class, "feeder");
            feeder.setPosition(REST_POS);
        } catch (Exception ignore) {
            feeder = null;
        }

        // Directions (your originals)
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU (your originals)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Flywheel controller
        fly = new FlywheelController(motorFlyWheel, TICKS_PER_REV);

        telemetry.addLine("GREEN BOT — with Flywheel RPM control + Distance map");
        telemetry.addLine("g2 Dpad Up/Down = change fake distance");
        telemetry.addLine("g2 RT = manual RPM scale (0..MAX_TEST_RPM) if you prefer");
        telemetry.addLine("g2 A = feed while at speed (if feeder servo exists)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ======= Drive (field-centric like your original) =======
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // Reset yaw / IMU same as before
            if (gamepad1.options) imu.resetYaw();
            if (gamepad1.back) imu.initialize(parameters);

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (gamepad1.left_bumper) {
                rx *= TURN_SPEED;
            }

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower  = (rotY + rotX +  rx) / denom;
            double backLeftPower   = (rotY - rotX +  rx) / denom;
            double frontRightPower = (rotY - rotX -  rx) / denom;
            double backRightPower  = (rotY + rotX -  rx) / denom;

            if (!(gamepad1.right_trigger > 0.1)) {
                frontLeftPower  *= MOVING_SPEED;
                backLeftPower   *= MOVING_SPEED;
                frontRightPower *= MOVING_SPEED;
                backRightPower  *= MOVING_SPEED;
            } else {
                frontLeftPower  *= MOVING_SPEED_FAST;
                backLeftPower   *= MOVING_SPEED_FAST;
                frontRightPower *= MOVING_SPEED_FAST;
                backRightPower  *= MOVING_SPEED_FAST;
            }
            if (!(gamepad1.left_trigger > 0.1)) {
                // already applied MOVING_SPEED above; keeping your structure
            } else {
                frontLeftPower  *= MOVING_SPEED_SLOW;
                backLeftPower   *= MOVING_SPEED_SLOW;
                frontRightPower *= MOVING_SPEED_SLOW;
                backRightPower  *= MOVING_SPEED_SLOW;
            }

            // Scootch moves (unchanged)
            if (gamepad1.dpad_left) {
                motorFrontLeft.setPower(-SCOOTCH_POWER);
                motorFrontRight.setPower( SCOOTCH_POWER);
                motorBackLeft.setPower(  SCOOTCH_POWER);
                motorBackRight.setPower(-SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_right) {
                motorFrontLeft.setPower( SCOOTCH_POWER);
                motorFrontRight.setPower(-SCOOTCH_POWER);
                motorBackLeft.setPower( -SCOOTCH_POWER);
                motorBackRight.setPower( SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_up) {
                motorFrontLeft.setPower( SCOOTCH_POWER);
                motorFrontRight.setPower( SCOOTCH_POWER);
                motorBackLeft.setPower(  SCOOTCH_POWER);
                motorBackRight.setPower( SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_down) {
                motorFrontLeft.setPower(-SCOOTCH_POWER);
                motorFrontRight.setPower(-SCOOTCH_POWER);
                motorBackLeft.setPower( -SCOOTCH_POWER);
                motorBackRight.setPower(-SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            // ======= Flywheel: choose target RPM =======
            // A) “Fake distance” at home: g2 dpad up/down adjusts test distance
            if (gamepad2.dpad_up && !dpadUpWas)   testDistanceFt += 0.25;
            if (gamepad2.dpad_down && !dpadDownWas) testDistanceFt = Math.max(0.5, testDistanceFt - 0.25);
            dpadUpWas = gamepad2.dpad_up;
            dpadDownWas = gamepad2.dpad_down;

            double rpmFromDistance = DistanceRpmMap.rpmFromFt(testDistanceFt);

            // B) Optional manual override via RT (comment out if you don’t want it)
            // If you just want to use distance mapping, set manualScale to -1.
            double manualScale = gamepad2.right_trigger; // 0..1
            double targetRPM = (manualScale > 0.05)
                    ? (manualScale * MAX_TEST_RPM)
                    : rpmFromDistance;

            // Future: when Limelight is hooked up, replace the targetRPM line with:
            // double distanceFt = getLimelightDistanceFt(); // your method
            // double targetRPM = DistanceRpmMap.rpmFromFt(distanceFt);

            // Command velocity and read back measured RPM
            fly.setTargetRpm(targetRPM);
            double measuredRPM = fly.getMeasuredRpm();
            boolean atSpeed = Math.abs(measuredRPM - targetRPM) <= RPM_TOL;

            // Optional feeder: hold A to feed only when at speed
            if (feeder != null) {
                if (atSpeed && gamepad2.a) feeder.setPosition(FEED_POS);
                else                       feeder.setPosition(REST_POS);
            }

            // ======= Telemetry (tight, readable) =======
            RobotLog.ii("DbgLog", "IMU Yaw(deg): " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Drive Pwr",  "FL:%.2f FR:%.2f BL:%.2f BR:%.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Fake Dist ft", "%.2f", testDistanceFt);
            telemetry.addData("Target RPM",   "%.0f", targetRPM);
            telemetry.addData("Measured RPM", "%.0f", measuredRPM);
            telemetry.addData("At Speed", atSpeed);
            telemetry.update();
        }
    }

    // === If you prefer, later you can drop in a Limelight distance fetch here ===
    // private double getLimelightDistanceFt() { /* read LL3A data → compute ft */ }
}
