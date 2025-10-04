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

@TeleOp(name = "LL3A → 3 Single LEDs (Patterns)", group = "Vision")
public class LL3A_ThreeSingleLEDs_TagPatterns extends LinearOpMode {

    // ===== CONFIG =====
    // If your LEDs turn ON when pin is LOW, set true:
    private static final boolean ACTIVE_LOW = true;
    private static final int STEP_MS = 200;

    // Device names must match DS config (Digital Device → Digital Device)
    private DigitalChannel led1green, led2green, led3green, led1red, led2red, led3red;

    // Limelight device name must match DS config (add "Limelight 3A")
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // LEDs
        led1green = hardwareMap.get(DigitalChannel.class, "led1green");
        led1red = hardwareMap.get(DigitalChannel.class, "led1red");
        
        led2green = hardwareMap.get(DigitalChannel.class, "led2green");
        led2red = hardwareMap.get(DigitalChannel.class, "led2red");
        
        led3green = hardwareMap.get(DigitalChannel.class, "led3green");
        led3red = hardwareMap.get(DigitalChannel.class, "led3red");
        
        led1green.setMode(DigitalChannel.Mode.OUTPUT);
        led1red.setMode(DigitalChannel.Mode.OUTPUT);
        
        led2green.setMode(DigitalChannel.Mode.OUTPUT);
        led2red.setMode(DigitalChannel.Mode.OUTPUT);
        
        led3green.setMode(DigitalChannel.Mode.OUTPUT);
        led3red.setMode(DigitalChannel.Mode.OUTPUT);
        //setAll(false,false,false);

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

            // --- CHOOSE 3-STEP PATTERN FOR (seen,id) ---
            if (id == 21){ //Green, Purple, Purple
                led1green.setState(true);
                led1red.setState(false);
                
                led2green.setState(false);
                led2red.setState(true);
                
                led3green.setState(false);
                led3red.setState(true);
            }
            else if (id == 22){ //Purple, Green, Purple
                led1green.setState(false);
                led1red.setState(true);
                
                led2green.setState(true);
                led2red.setState(false);
                
                led3green.setState(false);
                led3red.setState(true);
            }
            else if (id == 23){ //Purple, Purple, Green
                led1green.setState(false);
                led1red.setState(true);
                
                led2green.setState(false);
                led2red.setState(true);
                
                led3green.setState(true);
                led3red.setState(false);   
            }
            else{
                led1green.setState(false);
                led1red.setState(false);
                
                led2green.setState(false);
                led2red.setState(false);
                
                led3green.setState(false);
                led3red.setState(false);   
            }

            telemetry.addData("seen", seen);
            telemetry.addData("id", id);
            telemetry.addData("step", step);
            //telemetry.addData("led1/2/3", "%b %b %b", on1, on2, on3);
            telemetry.update();

            step = (step + 1) % 3;
            sleep(STEP_MS);
        }

        //setAll(false,false,false);
    }
}
