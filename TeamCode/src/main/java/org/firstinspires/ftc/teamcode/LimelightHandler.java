package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightHandler {

    private static Limelight3A limelight;

    public LimelightHandler() {
        // Does nothing until setupLimelight is called
    }
    public static void setupLimelight(HardwareMap hardwareMap, String deviceName, int initialPipeline) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(initialPipeline);
        limelight.start();
    }

    public static LLResult grabInformation() {
        if (limelight == null) {
            return null;
        }
        return limelight.getLatestResult();
    }

    public static LLStatus getStatus() {
        if (limelight == null) {
            return null;
        }
        return limelight.getStatus();
    }

    public static void shutdown() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}