package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Camera {
    private Limelight3A limelight;

    public Camera(HardwareMap hardwareMap, int aprilTagPipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipelineIndex);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public Map<Integer, Pose3D> getTagInfo3D() {
        Map<Integer, Pose3D> tagPositions = new HashMap<>();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                Pose3D pos = tag.getRobotPoseTargetSpace();
                tagPositions.put(id, pos);
            }
        }
        return tagPositions;
    }

    public Map<Integer, double[]> getTagInfo2D() {
        Map<Integer, double[]> tagAngles = new HashMap<>();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                double[] pos = new double[] { tag.getTargetXDegrees(), tag.getTargetYDegrees() };
                tagAngles.put(id, pos);
            }
        }
        return tagAngles;
    }
}
