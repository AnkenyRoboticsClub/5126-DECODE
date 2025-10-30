package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Drive Forward Auto")
public class SimpleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrain drive = new DriveTrain(hardwareMap); // same as TeleOp
        ImuUtil imu      = new ImuUtil(hardwareMap);    // same as TeleOp

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.driveStraightInches(this, 24.0, 0.5);               // forward 24 inches
        drive.turnToHeadingDegrees(this, imu, 90.0, 0.5, 0.015);  // face 90°
        drive.driveStraightInches(this, 12.0, 0.4);               // forward 12 inches

        telemetry.addLine("Auto Done");
        telemetry.update();
        sleep(250);
    }
}
