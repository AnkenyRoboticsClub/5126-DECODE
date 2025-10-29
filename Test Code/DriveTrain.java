package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

class DriveTrain {
    private final DcMotor fl, fr, bl, br;
    private final DcMotor flywheel;

    DriveTrain(HardwareMap hw) {
        fl = hw.dcMotor.get(Constants.M_FL);
        fr = hw.dcMotor.get(Constants.M_FR);
        bl = hw.dcMotor.get(Constants.M_BL);
        br = hw.dcMotor.get(Constants.M_BR);
        flywheel = hw.dcMotor.get(Constants.M_FLY);

        // Directions (match your original)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optionally set ZeroPowerBehavior, run modes, etc.
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br, flywheel}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** Field-centric drive. Inputs x,y,rx are gamepad values; headingRad from IMU. */
    void driveFieldCentric(double x, double y, double rx, double headingRad,
                           boolean slow, boolean fast) {
        // Rotate the input vector by -heading (field-oriented)
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        // Optional: scale turn speed like your original using trigger/bumper in OpMode
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double flP = (rotY + rotX + rx) / denominator;
        double blP = (rotY - rotX + rx) / denominator;
        double frP = (rotY - rotX - rx) / denominator;
        double brP = (rotY + rotX - rx) / denominator;

        double scale = Constants.MOVING_SPEED;       // default
        if (fast) scale = Constants.MOVING_SPEED_FAST;
        if (slow) scale = Constants.MOVING_SPEED_SLOW;

        fl.setPower(flP * scale);
        bl.setPower(blP * scale);
        fr.setPower(frP * scale);
        br.setPower(brP * scale);
    }

    void nudgeLeft(LinearOpMode op)  { nudge(-Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, op); }
    void nudgeRight(LinearOpMode op) { nudge( Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, op); }
    void nudgeForward(LinearOpMode op){ nudge( Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER,  Constants.SCOOTCH_POWER, op); }
    void nudgeBack(LinearOpMode op)  { nudge(-Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, -Constants.SCOOTCH_POWER, op); }

    private void nudge(double flP, double frP, double blP, double brP, LinearOpMode op) {
        fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);
        op.sleep(Constants.SCOOTCH_DURATION_MS);
    }

    void setFlywheel(boolean shoot) {
        flywheel.setPower(shoot ? 1.0 : 0.0);
    }
}
