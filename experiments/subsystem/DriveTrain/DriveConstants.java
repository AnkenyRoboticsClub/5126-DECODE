package org.firstinspires.ftc.teamcode;

final class DriveConstants {
    private Constants() {}

    // Drive speed scales, power # is from 0 -> 1, 0=0% and 1=100%
    static final double TURN_SPEED          = 0.5; //Prob should stay the same as default speed, honestly idk
    static final double MOVING_SPEED_SLOW   = 0.4; 
    static final double MOVING_SPEED        = 0.8; //Default Speed
    static final double MOVING_SPEED_FAST   = 1.0; //Usualy max power

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
}