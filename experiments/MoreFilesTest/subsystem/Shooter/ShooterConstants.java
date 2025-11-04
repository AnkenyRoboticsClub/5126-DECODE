package org.firstinspires.ftc.teamcode.subsystem.Shooter;

final class ShooterConstants {
    private Constants() {}

    // Shooter tuning
    static final double FLY_SPEED_SHOOT   = 1.0;  // forward shoot speed
    static final double FLY_SPEED_REVERSE = -0.5; // gentle reverse for unjam

    // Kicker positions (tune these!)
    static final double KICK_RETRACT = 0.00; // Down / hidden position
    static final double KICK_EXTEND  = 0.30; // Extended to eject balls
    static final long   KICK_TIME_MS = 250;  // How long to be extended for in ms

    static final String M_FLY = "motor1";//OG4
    static final String S_KICK = "kickServo";
    static final String M_INTAKE = "motorI";
}
