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
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void driveStraightInches(LinearOpMode op, double inches, double power) {
        resetDriveEncoders();
        int target = (int)Math.round(Math.abs(inchesToTicks(inches)));
        double dir = Math.signum(inches);
    
        fl.setPower(power * dir);
        fr.setPower(power * dir);
        bl.setPower(power * dir);
        br.setPower(power * dir);
    
        while (op.opModeIsActive() && averageAbsTicks() < target) {
            op.idle();
        }
        stopAll();
    }
    
    public void driveReverse(){
            fl.setPower(.5);
            bl.setPower(.5);
            fr.setPower(.5);
            br.setPower(.5);
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
