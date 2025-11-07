package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="GREEN BOT", group="Linear OpMode")
public class GREENBOT extends LinearOpMode {
    private DriveTrain drive;
    private ImuUtil imu;
    private Shooter shooter;

    @Override
    public void runOpMode() {
        drive   = new DriveTrain(hardwareMap);
        imu     = new ImuUtil(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision  = new VisionAlign(drive, imu);

        vision.start(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        boolean prevUnjam = false;
        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double heading = imu.getHeadingRad();

            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Drive
            drive.driveFieldCentric(x, y, rx, heading, slow, fast, false);


            // ---- Shooter controls (GP2 example) ----
            // RT: spin up; LB: quick reverse pulse; A: flick one ring
            if (gamepad2.right_trigger > 0.1) shooter.spinUp();
            else                              shooter.stop();

            if (gamepad2.right_bumper) shooter.intakeFW();// For temp human player feeding
            
            if (gamepad2.left_bumper) shooter.intake();
            else                      shooter.stopIntake();

            if (gamepad1.left_bumper)  drive.assistLeftt();
            if (gamepad1.right_bumper) drive.assistRight();

            if (gamepad2.a) shooter.feedOne(this); // extend + retract
            
            if (gamepad1.dpad_right) drive.nudgeRight();
            if (gamepad1.dpad_left)  drive.nudgeLeft();
            if (gamepad1.dpad_up)    drive.nudgeForward();
            if (gamepad1.dpad_down)  drive.nudgeBack();

            // while (A held) { vision.aimAndApproachStepRobotCentric(); }
            // on stop: vision.stop();
            
            //if (gamepad2.right_bumper && gamepad1.right_bumper) liftRobot();

            telemetry.addData("Flywheel", gamepad2.right_trigger > 0.1 ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
