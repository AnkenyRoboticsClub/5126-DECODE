package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "GREEN BOT", group = "Linear OpMode")
public class GREENBOT extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap);
        ImuUtil imu = new ImuUtil(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;   // note: Y is negative up on stick
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // Heading + IMU controls
            if (gamepad1.options) imu.resetYaw();
            if (gamepad1.back)    imu.reinit();

            double heading = imu.getHeadingRad();

            // Optional: match your TURN_SPEED behavior on LB (scale rx in place)
            if (gamepad1.left_bumper) {
                rx *= Constants.TURN_SPEED;  // same as your code (TURN_SPEED = 1.0 by default)
            }

            // Speed modes (same logic condensed)
            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Nudge (“scootch”) D-Pad
            if (gamepad1.dpad_left)  drive.nudgeLeft(this);
            if (gamepad1.dpad_right) drive.nudgeRight(this);
            if (gamepad1.dpad_up)    drive.nudgeForward(this);
            if (gamepad1.dpad_down)  drive.nudgeBack(this);

            // Flywheel on gamepad2 RT
            drive.setFlywheel(gamepad2.right_trigger > 0.1);

            // Field-centric drive output
            drive.driveFieldCentric(x, y, rx, heading, slow, fast);

            RobotLog.ii("DbgLog", "IMU Yaw (deg): %.1f", Math.toDegrees(heading));
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Speed Mode", slow ? "SLOW" : (fast ? "FAST" : "NORMAL"));
            telemetry.update();
        }
    }
}
