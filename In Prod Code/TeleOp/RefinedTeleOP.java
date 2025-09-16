package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "OpModeRed_Refined", group = "Linear OpMode")
public class OpModeRed_Refined extends LinearOpMode {

    // Speeds
    private static final double TURN_SLOW_FACTOR   = 0.35; // left bumper slow turn
    private static final double MOVING_SPEED_SLOW  = 0.40; // left trigger held
    private static final double MOVING_SPEED       = 0.90; // default cruise
    private static final double MOVING_SPEED_FAST  = 1.00; // right trigger held

    // Tiny “nudge” with D-pad (non-blocking)
    private static final double SCOOTCH_POWER = 0.30;   // vector magnitude to blend in
    private static final double SCOOTCH_BLEND = 0.25;   // how hard to blend into wheel powers

    // Deadband to ignore tiny stick noise
    private static final double DEADBAND = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Drive motors (make sure names match your config) ----
        DcMotor motorFrontLeft  = hardwareMap.dcMotor.get("Motor0");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Motor1");
        DcMotor motorBackLeft   = hardwareMap.dcMotor.get("Motor4");
        DcMotor motorBackRight  = hardwareMap.dcMotor.get("Motor5");

        // Zero-power behavior helps precise stops
        for (DcMotor m : new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Motor directions ---
        // Start with LEFT side REVERSE, RIGHT side FORWARD (common for direct-mount drivetrains).
        // If pushing stick forward yaw/spins, flip one entire side (don’t chase individual motors).
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---- IMU ----
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Set these to match the actual hub on your robot.
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,     // adjust if needed
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD   // adjust if needed
        ));
        imu.initialize(imuParams);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Reset yaw / re-init if needed
            if (gamepad1.options) imu.resetYaw();
            if (gamepad1.back)    imu.initialize(imuParams);

            // ---- Read sticks (apply deadband) ----
            double y  = deadband(-gamepad1.left_stick_y, DEADBAND);  // forward is +
            double x  = deadband( gamepad1.left_stick_x, DEADBAND);  // strafe  is +
            double rx = deadband( gamepad1.right_stick_x, DEADBAND); // turn    is +

            // Slow turning on left bumper
            if (gamepad1.left_bumper) {
                rx *= TURN_SLOW_FACTOR;
            }

            // ---- Field-centric transform ----
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // If your wheels were in "O" layout, you would do: rotX = -rotX; (yours are "X", so leave it)

            // ---- Mecanum mix ----
            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double fl = (rotY + rotX + rx) / denom;
            double bl = (rotY - rotX + rx) / denom;
            double fr = (rotY - rotX - rx) / denom;
            double br = (rotY + rotX - rx) / denom;

            // ---- Speed scale (single place) ----
            double speedScale;
            if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
                speedScale = MOVING_SPEED;          // tie-break: mid
            } else if (gamepad1.right_trigger > 0.1) {
                speedScale = MOVING_SPEED_FAST;     // sprint
            } else if (gamepad1.left_trigger > 0.1) {
                speedScale = MOVING_SPEED_SLOW;     // precision
            } else {
                speedScale = MOVING_SPEED;          // cruise
            }

            fl *= speedScale; bl *= speedScale; fr *= speedScale; br *= speedScale;

            // ---- Non-blocking D-pad “scootch” (adds a small vector) ----
            double nX = 0.0, nY = 0.0;
            if (gamepad1.dpad_left)  nX = -SCOOTCH_POWER;
            if (gamepad1.dpad_right) nX =  SCOOTCH_POWER;
            if (gamepad1.dpad_up)    nY =  SCOOTCH_POWER;
            if (gamepad1.dpad_down)  nY = -SCOOTCH_POWER;

            if (nX != 0.0 || nY != 0.0) {
                double nRotX = nX * Math.cos(-botHeading) - nY * Math.sin(-botHeading);
                double nRotY = nX * Math.sin(-botHeading) + nY * Math.cos(-botHeading);
                double d2 = Math.max(Math.abs(nRotY) + Math.abs(nRotX), 1.0);
                fl += (nRotY + nRotX) / d2 * SCOOTCH_BLEND;
                bl += (nRotY - nRotX) / d2 * SCOOTCH_BLEND;
                fr += (nRotY - nRotX) / d2 * SCOOTCH_BLEND;
                br += (nRotY + nRotX) / d2 * SCOOTCH_BLEND;
            }

            // ---- Apply powers ----
            motorFrontLeft.setPower(fl);
            motorBackLeft.setPower(bl);
            motorFrontRight.setPower(fr);
            motorBackRight.setPower(br);

            // ---- Telemetry ----
            telemetry.addData("Yaw (deg)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("Powers", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            telemetry.update();
        }
    }

    private static double deadband(double v, double band) {
        return (Math.abs(v) < band) ? 0.0 : v;
    }
}
