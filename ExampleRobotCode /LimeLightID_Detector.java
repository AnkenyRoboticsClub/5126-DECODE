// File: LL3A_ThreeSingleLEDs_TagPatterns.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// Limelight FTC API
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp(name = "LL3A → ID detector", group = "Vision")
public class LimeLightIDtester extends LinearOpMode {

    // ===== CONFIG =====
    private static final int STEP_MS = 200;

    // Device names must match DS config (Digital Device → Digital Device)

    // Limelight device name must match DS config (add "Limelight 3A")
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // must match config name
        limelight.setPollRateHz(50);
        
        limelight.start(); // begin publishing results
        telemetry.addLine("Init OK — press START");
        telemetry.update();
        waitForStart();

        int step = 0;

        while (opModeIsActive()) {
            // --- READ TAG FROM LL3A ---
            LLResult r = limelight.getLatestResult();
            limelight.pipelineSwitch(0); // Switch to pipeline number 0
            boolean seen = (r != null && r.isValid());
            int id = -1;
            if (seen) {
                List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
                if (tags != null && !tags.isEmpty()) id = tags.get(0).getFiducialId();
                else seen = false;
            }
            

            telemetry.addData("seen", seen);
            telemetry.addData("id", id);
            telemetry.addData("step", step);
            telemetry.update();

            step = (step + 1) % 3;
            sleep(STEP_MS);
        }
    }
}
