package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Te")
public class SpecAuto2ct extends LinearOpMode {

  private DcMotor motorBackLeft;
  private DcMotor motorFrontLeft;
  private DcMotor motorBackRight;
  private DcMotor motorFrontRight;

  int timeBetweenChecks;
  int sleepTime;
  double speed;

  /**
  * This function is executed when this OpMode is selected from the Driver Station.
  */
  @Override
  public void runOpMode() {
    motorBackLeft = hardwareMap.get(DcMotor.class, "Motor0");
    motorFrontLeft = hardwareMap.get(DcMotor.class, "Motor1");
    motorBackRight = hardwareMap.get(DcMotor.class, "Motor3");
    motorFrontRight = hardwareMap.get(DcMotor.class, "Motor2");

    motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    
    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // Time in ms between encoder checks
    timeBetweenChecks = 10;
    // time to wait between moves
    sleepTime = 500;
    // Speed to set the motors, OG speed = 1
    speed = 1;
    
    telemetry.addData("Action", "Robot Ready");
    telemetry.update();
    //waitForStart();
    
  boolean debugMode = false; // Set to true for testing, false for competition
  
  if (!debugMode) {
    waitForStart();
  } else {
    telemetry.addData("Status", "Debug Mode Active - Skipping waitForStart()");
    telemetry.update();
  }
    move("forward", 200);
  }

  /**
  * with movement
  */
  
  private void move(String direction, int distance) {
    // Current Available moves - forward, backward, left, right, rotateLeft, rotateRight
    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    viperRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    viperLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    viperRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    viperLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    if (direction.equals("forward")) {
      if(!close){
        motorBackLeft.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorFrontRight.setPower(speed);
      }
      else{
        motorBackLeft.setPower(verySpeed);
        motorFrontLeft.setPower(verySpeed);
        motorBackRight.setPower(verySpeed);
        motorFrontRight.setPower(verySpeed);
      }
      
      while (motorBackLeft.getCurrentPosition() <= distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("backward")) {
      if(!close){
        motorBackLeft.setPower(-speed);
        motorFrontLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
        motorFrontRight.setPower(-speed);
      }
      else{
        motorBackLeft.setPower(-verySpeed);
        motorFrontLeft.setPower(-verySpeed);
        motorBackRight.setPower(-verySpeed);
        motorFrontRight.setPower(-verySpeed);
      }
      
      while (motorBackLeft.getCurrentPosition() >= -distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("left")) {
      motorBackLeft.setPower(speed);
      motorFrontLeft.setPower(-speed);
      motorBackRight.setPower(-speed);
      motorFrontRight.setPower(speed);
      
      while (motorBackLeft.getCurrentPosition() <= distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("right")) {
      motorBackLeft.setPower(-speed);
      motorFrontLeft.setPower(speed);
      motorBackRight.setPower(speed);
      motorFrontRight.setPower(-speed);
      
      while (motorBackLeft.getCurrentPosition() >= -distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("rotateLeft")) {
      motorBackLeft.setPower(-speed);
      motorFrontLeft.setPower(-speed);
      motorBackRight.setPower(speed);
      motorFrontRight.setPower(speed);
      
      while (motorBackLeft.getCurrentPosition() >= -distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("rotateRight")) {
      motorBackLeft.setPower(speed);
      motorFrontLeft.setPower(speed);
      motorBackRight.setPower(-speed);
      motorFrontRight.setPower(-speed);
      
      while (motorBackLeft.getCurrentPosition() <= distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("diagonalRight")) {
      motorBackLeft.setPower(0);
      motorFrontLeft.setPower(speed);
      motorBackRight.setPower(speed);
      motorFrontRight.setPower(0);
      
      while (motorFrontRight.getCurrentPosition() <= distance) {
        sleep(timeBetweenChecks);
      }
    }
    if (direction.equals("diagonalLeft")) {
      motorBackLeft.setPower(speed);
      motorFrontLeft.setPower(0);
      motorBackRight.setPower(0);
      motorFrontRight.setPower(speed);
      
      while (motorBackLeft.getCurrentPosition() <= distance) {
        sleep(timeBetweenChecks);
      }
    }
    
    motorBackLeft.setPower(0);
    motorFrontLeft.setPower(0);
    motorBackRight.setPower(0);
    motorFrontRight.setPower(0);
    
    sleep(sleepTime);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }
}


