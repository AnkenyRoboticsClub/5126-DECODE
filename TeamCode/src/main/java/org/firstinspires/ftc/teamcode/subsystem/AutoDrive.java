package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

public class AutoDrive {
    private final DcMotor fl, fr, bl, br;

    public AutoDrive(HardwareMap hw) {
        fl = hw.dcMotor.get(RobotConstants.M_FL);
        fr = hw.dcMotor.get(RobotConstants.M_FR);
        bl = hw.dcMotor.get(RobotConstants.M_BL);
        br = hw.dcMotor.get(RobotConstants.M_BR);

        // Directions (match your original)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    //=======================AUTO=====================
    // --- tune these at the top of DriveTrain or near your fields ---
    static final double TICKS_PER_REV = 28;   // set for your motor
    static final double WHEEL_DIAMETER_IN = 3.78;// set for your wheel
    static final double GEAR_RATIO = 1.0;        // output gear ratio to wheel
    
    private double inchesToTicks(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER_IN;
        double revs = (inches / circumference) * GEAR_RATIO;
        return revs * TICKS_PER_REV;
    }
    
    public void resetDriveEncoders() {
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    public int averageAbsTicks() {
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())
               + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
    }
    
    public void stopAll() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }
    
    public void aidenTurn(){
        fl.setPower(.6); fr.setPower(-.4); bl.setPower(-.4); br.setPower(.6);
    }

    /** Drive straight (robot-centric) for inches at given power using encoders. */
    public void driveStraightInches(LinearOpMode op, double inches, double maxPower) {
        resetDriveEncoders();

        // Convert inches → ticks using RobotConstants
        int targetTicks = (int) Math.round(inches * RobotConstants.TICKS_PER_INCH);
        double direction = Math.signum(inches);

        maxPower = Math.abs(maxPower) * direction;

        double minPower = 0.12;
        double accelDist = 0.25;
        double decelDist = 0.25;

        while (op.opModeIsActive()) {

            int current = averageAbsTicks();
            int absTarget = Math.abs(targetTicks);

            if (current >= absTarget) break;

            double progress = (double) current / absTarget;

            double commandedPower;

            if (progress < accelDist) {
                double scale = progress / accelDist;
                commandedPower = lerp(minPower, maxPower, scale);
            }
            else if (progress < 1.0 - decelDist) {
                commandedPower = maxPower;
            }
            else {
                double scale = (1.0 - progress) / decelDist;
                commandedPower = lerp(minPower, maxPower, scale);
            }

            fl.setPower(commandedPower);
            fr.setPower(commandedPower);
            bl.setPower(commandedPower);
            br.setPower(commandedPower);

            op.idle();
        }
        stopAll();
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    
    public void driveReverse(){
            fl.setPower(-.5);
            bl.setPower(-.5);
            fr.setPower(-.5);
            br.setPower(-.5);
    }
    
    /** Turn in place to an absolute heading (deg, -180..180) using IMU (simple P). */
    public void turnToHeadingDegrees(LinearOpMode op, ImuUtil imu, double targetDeg, double maxPower, double kP) {
        while (op.opModeIsActive()) {
            double currentDeg = Math.toDegrees(imu.getHeadingRad());
            double error = angleWrapDeg(targetDeg - currentDeg);
            if (Math.abs(error) < 1.5) break;
    
            double turn = kP * error;
            if (turn >  maxPower) turn =  maxPower;
            if (turn < -maxPower) turn = -maxPower;
    
            fl.setPower( turn); bl.setPower( turn);
            fr.setPower(-turn); br.setPower(-turn);
            op.idle();
        }
        stopAll();
    }
    
    private double angleWrapDeg(double a) {
        while (a > 180)  a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
    
}
