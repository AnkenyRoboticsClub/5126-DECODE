package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp (name="OpModeRed", group="Linear OpMode")
public class OpModeRed extends LinearOpMode {
    // You can modify these to make thing go faster or slower
    static final double TURN_SPEED = 1.0;
    static final double MOVING_SPEED_SLOW = 0.4;
    static final double MOVING_SPEED = 0.9;
    static final double MOVING_SPEED_FAST = 1.0;
    private static final double SCOOTCH_POWER = 0.3; // Adjust power as needed
    private static final long SCOOTCH_DURATION_MS = 200; // Adjust duration as needed 
    
    
    // carefull to modify andy thing down V
   
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Motor1");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Motor0");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Motor2");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Motor3");

        //Reverse the motors that are reversed
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // for connors current config
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        // Wait for the start button

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            
            //Reset Yaw V
            if (gamepad1.options) {
                imu.resetYaw();
            }
            // Reset IMU V
            if (gamepad1.back){
                imu.initialize(parameters);
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            //Slow Turn V
            if(gamepad1.left_bumper){
                rx *= TURN_SPEED;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + (rx)) / denominator;
            double backLeftPower = (rotY - rotX + (rx)) / denominator;
            double frontRightPower = (rotY - rotX - (rx)) / denominator;
            double backRightPower = (rotY + rotX - (rx)) / denominator;
           

            if(!(gamepad1.right_trigger > 0.1)) // Trigger Not pressed = Normal mode
            {
                frontLeftPower *= MOVING_SPEED;
                backLeftPower *= MOVING_SPEED;
                frontRightPower *= MOVING_SPEED;
                backRightPower *= MOVING_SPEED;
            }
            if((gamepad1.right_trigger > 0.1)) // Speed is increased when Right trigger pulled
            {
                frontLeftPower *= MOVING_SPEED_FAST;
                backLeftPower *= MOVING_SPEED_FAST;
                frontRightPower *= MOVING_SPEED_FAST;
                backRightPower *= MOVING_SPEED_FAST;
            }
            if(!(gamepad1.left_trigger > 0.1)) // Trigger Not pressed = Normal mode
            {
                frontLeftPower *= MOVING_SPEED;
                backLeftPower *= MOVING_SPEED;
                frontRightPower *= MOVING_SPEED;
                backRightPower *= MOVING_SPEED;
            }
            if((gamepad1.left_trigger > 0.1)) // Speed is Decreased when Left trigger pulled
            {
                frontLeftPower *= MOVING_SPEED_SLOW;
                backLeftPower *= MOVING_SPEED_SLOW;
                frontRightPower *= MOVING_SPEED_SLOW;
                backRightPower *= MOVING_SPEED_SLOW;
            }
            if (gamepad1.dpad_left) // the robot will move just the slightese bit to the left
            {
                motorFrontLeft.setPower(-SCOOTCH_POWER);
                motorFrontRight.setPower(SCOOTCH_POWER);
                motorBackLeft.setPower(SCOOTCH_POWER);
                motorBackRight.setPower(-SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_right) // the robot will move just the slightese bit to the right
            {
                motorFrontLeft.setPower(SCOOTCH_POWER);
                motorFrontRight.setPower(-SCOOTCH_POWER);
                motorBackLeft.setPower(-SCOOTCH_POWER);
                motorBackRight.setPower(SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_up) // the robot will move just the slightese bit to the right
            {
                motorFrontLeft.setPower(SCOOTCH_POWER);
                motorFrontRight.setPower(SCOOTCH_POWER);
                motorBackLeft.setPower(SCOOTCH_POWER);
                motorBackRight.setPower(SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            if (gamepad1.dpad_down) // the robot will move just the slightese bit to the right
            {
                motorFrontLeft.setPower(-SCOOTCH_POWER);
                motorFrontRight.setPower(-SCOOTCH_POWER);
                motorBackLeft.setPower(-SCOOTCH_POWER);
                motorBackRight.setPower(-SCOOTCH_POWER);
                sleep(SCOOTCH_DURATION_MS);
            }
            
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
            RobotLog.ii("DbgLog", "IMU Expansion: " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Motor","%.2f, %.2f, %.2f, %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}