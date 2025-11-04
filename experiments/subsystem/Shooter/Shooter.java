package org.firstinspires.ftc.teamcode.subsystem.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotor fly;
    private final Servo kick;
    private final DcMotor intake;

    public Shooter(HardwareMap hw) {
        fly  = hw.dcMotor.get(ShooterConstants.M_FLY);
        kick = hw.servo.get(ShooterConstants.S_KICK);
        intake = hw.dcMotor.get(ShooterConstants.M_INTAKE);

        // Match the direction you used before so +power = shoot
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Safe default
        fly.setPower(0);
        intake.setPower(0);
        kick.setPosition(ShooterConstants.KICK_RETRACT);
    }

    // ----- Flywheel controls -----
    public void setFlywheelPower(double p) {
        if (p > 1) p = 1;
        if (p < -1) p = -1;
        fly.setPower(p);
    }
    public void spinUp()  { setFlywheelPower(ShooterConstants.FLY_SPEED_SHOOT); }
    public void stop()    { setFlywheelPower(0); }
    public void intakeFW(){ setFlywheelPower(ShooterConstants.FLY_SPEED_REVERSE); }
    public void intake()  { intake.setPower(1); }
    public void stopIntake() {intake.setPower(0); }

    // ----- Kicker controls -----
    public void setKicker(boolean extended) {
        kick.setPosition(extended ? ShooterConstants.KICK_EXTEND : ShooterConstants.KICK_RETRACT);
    }
    /** One flick: extend, wait, retract. */
    public void flick(LinearOpMode op) {
        setKicker(true);
        op.sleep(ShooterConstants.KICK_TIME_MS);
        setKicker(false);
    }

    // ----- Convenience combos -----
    /** Typical feed sequence: (optionally spinUp before calling) */
    public void feedOne(LinearOpMode op) {
        // assumes flywheel is already up to speed
        flick(op);
    }
}
