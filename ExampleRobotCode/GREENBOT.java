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


@TeleOp (name="GREEN BOT", group="Linear OpMode")
public class GREENBOT extends LinearOpMode {
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
        

        /*=============READ THIS IF ROBOT IS DRIVING WEIRD=================
            The weird boxes down below is supposed to be representing the 6-pack
            motor design, and if you change any please be sure to change it on here

            DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor0");
               ^            ^                                   ^
               |            |                                   |
            This just   | This is the     |             This is the part
            is the motor|motor name,      | that will mostlikely need changed, its 
            type        | it shouldnt need| just what the code looks for in the driver station, 
            Dont change | changed         | so whatever motor you name "motor0" the code will see it 
                                          | and control that one as Front Left
        */
        /*
                         Front
                           ^
                           |
            Wheel   [motor0][motor1]  Wheel
          Fly Wheel [motor4][motor5]   N/A
            Wheel   [motor2][motor3]  Wheel 
        
        */

        //=======The motor# part is most likely messed up====v===
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor0");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor3");
        //=======================================================

        DcMotor motorFlyWheel = hardwareMap.dcMotor.get("motor4"); //<-- i know this is correct

        //Reverse the motors that are reversed, im like 85% sure these are correct
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, // for connors current config
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
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


            //======THIS IS FLYWHEEL POWER=======
            if(gamepad2.right_trigger > 0.1){ 
                motorFlyWheel.setPower(1);
            }
            else{
                motorFlyWheel.setPower(0);
            }
            //Basicly its if it detects any trigger input
            //It will spin motor4 aka the flywheel motor to full power
            //the power number in code if from 0-1 
            // 0 being 0% and 1 being 100% power
            // and the number can be as fine as you want in terms 
            // of decimal point, aka .25 = 25% or .75 =75% 


            //===================================================================================================================
            //EVERYTHING BELOW SHOULD WORK, SO PROBLY DONT TOUCH
            //===================================================================================================================


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