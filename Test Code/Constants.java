package org.firstinspires.ftc.teamcode;

final class Constants {
    private Constants() {}

    // Drive speed scales
    static final double TURN_SPEED          = 1.0;
    static final double MOVING_SPEED_SLOW   = 0.4;
    static final double MOVING_SPEED        = 0.9;
    static final double MOVING_SPEED_FAST   = 1.0;

    // Nudge (“scootch”)
    static final double SCOOTCH_POWER       = 0.3;   // tune
    static final long   SCOOTCH_DURATION_MS = 200;   // tune

    // Motor names (edit if your config names change)
    static final String M_FL = "motor0";
    static final String M_FR = "motor1";
    static final String M_BL = "motor2";
    static final String M_BR = "motor3";
    static final String M_FLY = "motor4";

    // IMU name
    static final String IMU = "imu";
}
