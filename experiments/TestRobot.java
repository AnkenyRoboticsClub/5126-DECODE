package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="Test Robot", group="Linear OpMode")
public class TestRobot extends LinearOpMode {
    private DriveTrain drive;
    private ImuUtil imu;
    private Shooter shooter;

    @Override
    public void runOpMode() {
        drive   = new DriveTrain(hardwareMap);
        imu     = new ImuUtil(hardwareMap);
        shooter = new Shooter(hardwareMap);
        //vision  = new VisionAlign(drive, imu);

        //vision.start(hardwareMap);

        int option = 1;
        boolean prevLeftBumper  = false;
        boolean prevRightBumper = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double heading = imu.getHeadingRad();

            boolean fast = gamepad1.right_trigger > 0.1;
            boolean slow = gamepad1.left_trigger  > 0.1;

            // Detect rising edge (button just pressed)
            if (gamepad1.right_bumper && !prevRightBumper) {
                option++;
                if (option > 3) option = 1;
            } else if (gamepad1.left_bumper && !prevLeftBumper) {
                option--;
                if (option < 1) option = 3;
            }
        
            prevRightBumper = gamepad1.right_bumper;
            prevLeftBumper  = gamepad1.left_bumper;
        
            // Now display the current menu
            telemetry.clearAll();
            telemetry.addData("Option ", option);
            telemetry.addLine("Use bumpers to move selection");


            //Option 1: Drive train, Option 2: Shooting system, Option 3: Vision System
            if (option == 1){//Drive Train

                drive.driveFieldCentric(x, y, rx, heading, slow, fast, false);

                if (gamepad1.dpad_right) drive.nudgeRight();
                if (gamepad1.dpad_left)  drive.nudgeLeft();
                if (gamepad1.dpad_up)    drive.nudgeForward();
                if (gamepad1.dpad_down)  drive.nudgeBack();

                if (gamepad1.a) drive.testDrive();


                telemetry.addLine("Drive Train");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks - Left: Forward & Backward Right: Turning");
                telemetry.addLine("Left  Trigger  - Slow Mode");
                telemetry.addLine("Right Trigger  - Fast Mode");
                telemetry.addLine("Gampad A  - Test Drive");
                telemetry.addLine("Gampad B  -");
                telemetry.addLine("Gampad X  -");
                telemetry.addLine("Gampad Y  -");
                telemetry.addLine("Dpads - Scoot");
                telemetry.update();
            }
            else if (option == 2){

                if (gamepad1.right_trigger > 0.1) shooter.spinUp();
                else                              shooter.stop();

                if (gamepad1.left_trigger > 0.1) shooter.intakeFW();

                if (gamepad1.b) shooter.intake();
                else            shooter.stopIntake();

                if (gamepad2.a) shooter.feedOne(this);


                telemetry.addLine("Shooting System");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks -");
                telemetry.addLine("Left  Trigger  - Reverse Spin");
                telemetry.addLine("Right Trigger  - Spin up fly wheel");
                telemetry.addLine("Gampad A  - Feed a ball via servo");
                telemetry.addLine("Gampad B  - Intake (If installed)");
                telemetry.addLine("Gampad X  -");
                telemetry.addLine("Gampad Y  -");
                telemetry.addLine("Dpads - Power:(To be implemented)");
                telemetry.update();
            }
            else if (option == 3){

                if (gamepad1.a) vision.aimStepRobotCentric();
                if (gamepad1.b) vision.aimAndApproachStepRobotCentric();

                telemetry.addLine("Vison System");
                telemetry.addLine("!!!NOT IMPLEMENTED YET!!!");
                telemetry.addLine("========================================");
                telemetry.addLine("Joysticks - ");
                telemetry.addLine("Left  Trigger  -");
                telemetry.addLine("Right Trigger  -");
                telemetry.addLine("Gampad A  - Turn to center the tag");
                telemetry.addLine("Gampad B  - Turn + creep forward toward standoff");
                telemetry.addLine("Gampad X  -");
                telemetry.addLine("Gampad Y  -");
                telemetry.addLine("Dpads -");
                telemetry.update();
            }
            else{
                drive.stopAll();
                shooter.stop();
                shooter.stopIntake();

                telemetry.addLine("There has been an error, please restart the robot");
                telemetry.addLine("And then tell Jayden how you made the error happen :)");
                telemetry.update();

            }
        }
    }
}
