package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

class DriveTrain {
    private final DcMotor fl, fr, bl, br;

    // Slew state (what we actually send to motors)
    private double flCmd = 0, frCmd = 0, blCmd = 0, brCmd = 0;

    DriveTrain(HardwareMap hw) {
        fl = hw.dcMotor.get(Constants.M_FL);
        fr = hw.dcMotor.get(Constants.M_FR);
        bl = hw.dcMotor.get(Constants.M_BL);
        br = hw.dcMotor.get(Constants.M_BR);

        // Directions (match your original)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // open-loop by default; velocity control can be added later
        }
    }

    /** Smoothly move current -> target by at most SLEW_PER_LOOP each loop. */
    private double slew(double current, double target) {
        double d = target - current;
        double s = Constants.SLEW_PER_LOOP;
        if (d >  s) d =  s;
        if (d < -s) d = -s;
        return current + d;
    }

    /** Optional helper to clear slew memory (e.g., before/after nudges). */
    void resetCommands() { flCmd = frCmd = blCmd = brCmd = 0; }

    /** Field-centric mecanum. x,y,rx are joystick inputs; headingRad from IMU. */
    void driveFieldCentric(double x, double y, double rx, double headingRad,
                           boolean slow, boolean fast, boolean rwd) {

        // Rotate input vector by -heading (field oriented)
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        // Mecanum mix (unitless -1..1)
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double flP = (rotY + rotX + rx) / denom;
        double blP = (rotY - rotX + rx) / denom;
        double frP = (rotY - rotX - rx) / denom;
        double brP = (rotY + rotX - rx) / denom;

        // Speed mode from your Constants
        double scale = Constants.SPEED_NORMAL;
        if (fast) scale = Constants.SPEED_FAST;
        if (slow) scale = Constants.SPEED_SLOW;

        // Input shaping for low-speed finesse; then scale
        flP = Math.copySign(flP * flP, flP) * scale;
        frP = Math.copySign(frP * frP, frP) * scale;
        blP = Math.copySign(blP * blP, blP) * scale;
        brP = Math.copySign(brP * brP, brP) * scale;

        // Slew to avoid jerky starts / wheel slip
        if (rwd) {
            blCmd = slew(blCmd, blP);
            brCmd = slew(brCmd, brP);
            // rear-wheel drive only
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(blCmd);
            br.setPower(brCmd);
        } else {
            flCmd = slew(flCmd, flP);
            frCmd = slew(frCmd, frP);
            blCmd = slew(blCmd, blP);
            brCmd = slew(brCmd, brP);

            fl.setPower(flCmd);
            fr.setPower(frCmd);
            bl.setPower(blCmd);
            br.setPower(brCmd);
        }
    }

    /** Robot-centric mecanum (no field rotation). Uses normal speed & slew. */
    void driveRobot(double x, double y, double rx) {
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flP = (y + x + rx) / denom;
        double blP = (y - x + rx) / denom;
        double frP = (y - x - rx) / denom;
        double brP = (y + x - rx) / denom;

        double scale = Constants.SPEED_NORMAL;
        flP = Math.copySign(flP * flP, flP) * scale;
        frP = Math.copySign(frP * frP, frP) * scale;
        blP = Math.copySign(blP * blP, blP) * scale;
        brP = Math.copySign(brP * brP, brP) * scale;

        flCmd = slew(flCmd, flP);
        frCmd = slew(frCmd, frP);
        blCmd = slew(blCmd, blP);
        brCmd = slew(brCmd, brP);

        fl.setPower(flCmd);
        fr.setPower(frCmd);
        bl.setPower(blCmd);
        br.setPower(brCmd);
    }

    // Quick “taps” for alignment — bypass slew for a crisp move, then stop/clear
    void nudgeLeft(LinearOpMode op)   { nudge(-Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, op); }
    void nudgeRight(LinearOpMode op)  { nudge( Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, op); }
    void nudgeForward(LinearOpMode op){ nudge( Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, op); }
    void nudgeBack(LinearOpMode op)   { nudge(-Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, op); }

    private void nudge(double flP, double frP, double blP, double brP, LinearOpMode op) {
        fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);
        op.sleep(Constants.SCOOTCH_DURATION_MS);
        stopAll();
        resetCommands();
    }

    void stopAll() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }

    void assistRight(){
        double p = Constants.SPEED_NORMAL; // or Constants.TURN_SPEED if you add one
        fl.setPower( p); fr.setPower(-p); bl.setPower(-p); br.setPower( p);
        flCmd =  p; frCmd = -p; blCmd = -p; brCmd =  p;
    }
    void assistLeft(){
        double p = Constants.SPEED_NORMAL; // or Constants.TURN_SPEED if you add one
        fl.setPower(-p); fr.setPower( p); bl.setPower( p); br.setPower(-p);
        flCmd = -p; frCmd =  p; blCmd =  p; brCmd = -p;
    }

    void testDrive(){
        fl.setPower(-.2); fr.setPower( .2); bl.setPower( .2); br.setPower(-.2);
    }
}
