package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

class ImuUtil {
    private final IMU imu;
    private final IMU.Parameters params;

    //Make sure to tune the directions when you move the control hub
    ImuUtil(HardwareMap hw) {
        imu = hw.get(IMU.class, Constants.IMU);
        params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(params);
    }

    void resetYaw() { imu.resetYaw(); }
    void reinit()   { imu.initialize(params); }

    double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    IMU getRaw() { return imu; } // optional, for logging
}
