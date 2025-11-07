package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Shoot 1")
public class Shooting1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoDrive drive = new AutoDrive(hardwareMap); // same as TeleOp
        Shooter shooter =  new Shooter(hardwareMap);    // same as TeleOp
        ImuUtil imu      = new ImuUtil(hardwareMap);    // same as TeleOp
        VisionAlign vision = new VisionAlign(drive, imu);
        
        // after constructing imu, before waitForStart:
        imu.resetYaw();
        vision.start(hardwareMap);

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        vision.aimAndApproachUntil(this);
        shooter.spinUp();
        sleep(1000);
        shooter.feedOne(this);
        shooter.stop();

        vision.stop();
        
        telemetry.addLine("Auto Done");
        telemetry.update();
        sleep(250);
    }
}
