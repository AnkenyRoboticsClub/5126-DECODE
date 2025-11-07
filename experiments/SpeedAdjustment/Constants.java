package org.firstinspires.ftc.teamcode;

final class Constants {
    private Constants() {}

    // Drive speed scales, power # is from 0 -> 1, 0=0% and 1=100%
    static final double SLEW_PER_LOOP = 0.06; // change per 20ms; tune 0.03–0.08
    static final double SPEED_NORMAL  = 0.65;
    static final double SPEED_FAST    = 0.90;
    static final double SPEED_SLOW    = 0.40;

    // Shooter tuning
    static final double FLY_SPEED_SHOOT   = 1.0;  // forward shoot speed
    static final double FLY_SPEED_REVERSE = -0.5; // gentle reverse for unjam

    // Kicker positions (tune these!)
    static final double KICK_RETRACT = 0.00; // Down / hidden position
    static final double KICK_EXTEND  = 0.30; // Extended to eject balls
    static final long   KICK_TIME_MS = 250;  // How long to be extended for in ms

    // Nudge (“scootch”)
    static final double SCOOTCH_POWER       = 0.3;   // tune for more powerfull scootch
    static final long   SCOOTCH_DURATION_MS = 200;   // tune for longer or shorter scootch

    // Motor names (edit if your config names change)
    static final String M_FL = "motor0";
    static final String M_FR = "motor2";
    static final String M_BL = "motor4";//OG2
    static final String M_BR = "motor3";
    
    static final String M_FLY = "motor1";//OG4
    static final String S_KICK = "kickServo";
    static final String M_INTAKE = "motorI";
    
    //ViperCode V
    //static final String M_VL = "motorVL";
    //static final String M_VR = "motorVR";

    // IMU name
    static final String IMU = "imu";

    // ===================== Limelight 4A (USB-C via FTC SDK) =====================
    // Name in the RC configuration (Driver Station -> Configure Robot)
    static final String LL_DEVICE_NAME     = "limelight";  // match your config name
    
    // Desired stand-off (use ta as a rough distance proxy). Set after you observe it.
    static final double LL_TARGET_AREA     = 8.0;
    
    // Aim (turn) tuning
    static final double LL_K_TURN          = 0.018;  // P on tx (deg -> turn power)
    static final double LL_MAX_TURN        = 0.45;   // clamp
    static final double LL_MIN_TURN        = 0.06;   // small feed-forward
    static final double LL_AIM_TOL_DEG     = 1.5;    // "good enough" aim error
    
    // Approach (forward) tuning
    static final double LL_K_FORWARD       = 0.020;  // P on (targetArea - ta)
    static final double LL_MAX_FORWARD     = 0.35;
    static final double LL_MIN_FORWARD     = 0.05;
    static final double LL_APPROACH_TOL_TA = 0.5;    // how close ta must be to targetArea
    
    // Auto safety timeouts
    static final double LL_ALIGN_TIMEOUT_S   = 2.0;
    static final double LL_APPROACH_TIMEOUT_S= 2.5;

}
