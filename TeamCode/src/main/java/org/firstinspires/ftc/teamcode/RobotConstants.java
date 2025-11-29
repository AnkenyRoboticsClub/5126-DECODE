package org.firstinspires.ftc.teamcode;

final class RobotConstants {
    private RobotConstants() {}

    // Drive speed scales, power # is from 0 -> 1, 0=0% and 1=100%
    static final double TURN_SPEED          = 0.7; //Prob should stay the same as default speed, honestly idk
    static final double MOVING_SPEED_SLOW   = 0.4; 
    static final double MOVING_SPEED        = 0.6; //Default Speed
    static final double MOVING_SPEED_FAST   = 1.0; //Usualy max power

    // Shooter tuning
    static final double FLY_SPEED_SHOOT   = 1.0;  // forward shoot speed
    static final double FLY_SPEED_REVERSE = -0.5; // gentle reverse for unjam

    // Kicker positions (tune these!)
    static final double KICK_RETRACT = -0.10; // Down / hidden position
    static final double KICK_EXTEND  = 0.30; // Extended to eject balls
    static final long   KICK_TIME_MS = 250;  // How long to be extended for in ms

    // Nudge (“scootch”)
    static final double SCOOTCH_POWER       = 0.3;   // tune for more powerfull scootch
    static final long   SCOOTCH_DURATION_MS = 200;   // tune for longer or shorter scootch

    // Motor names (edit if your config names change)
    static final String M_FL = "motor3";
    static final String M_FR = "motor2";//Good
    static final String M_BL = "motor1";
    static final String M_BR = "motor0";//Good
    
    static final String M_FLY = "motor4";//OG4
    static final String S_KICK = "kickServo";
    static final String M_INTAKE = "motorI";
    
    //ViperCode V
    //static final String M_VL = "motorVL";
    //static final String M_VR = "motorVR";

    // IMU name
    static final String IMU = "imu";

    // -------------------- LIMELIGHT CONSTANTS --------------------

    // Device name from RC config
    public static final String LL_DEVICE_NAME = "limelight";

    // Tuning multipliers
    public static final double LL_K_TURN      = 0.035;   // turning gain
    public static final double LL_K_FORWARD   = 0.25;    // forward gain

    // Minimum motor power so robot “breaks static friction”
    public static final double LL_MIN_TURN    = 0.10;
    public static final double LL_MIN_FORWARD = 0.10;

    // Maximum motor speeds for LL control
    public static final double LL_MAX_TURN    = 0.5;
    public static final double LL_MAX_FORWARD = 0.35;

    // TA value when robot is at perfect shooting distance
// ★ YOU MUST UPDATE THIS using LLDebug ★
    public static double LL_TARGET_AREA = 1.45;

    // Tolerances for stopping
    public static final double LL_AIM_TOL_DEG      = 1.5;
    public static final double LL_APPROACH_TOL_TA  = 0.05;

    // Auto alignment timeouts
    public static final double LL_ALIGN_TIMEOUT_S    = 2.0;
    public static final double LL_APPROACH_TIMEOUT_S = 3.0;
}
