package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Simple AprilTag webcam wrapper (mirrors the YouTube tutorial class).
 * Uses MJPEG stream format (30 FPS, no YUV2 warning) and streams to FTC Dashboard.
 */
public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> currentDetections;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)   // MJPEG = 30 FPS, no YUV2 warning
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .build();

        // Stream camera feed to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    /** Call once per loop iteration to refresh detections. */
    public void update() {
        currentDetections = aprilTagProcessor.getDetections();
    }

    /** Get an AprilTag detection by its specific ID, or null if not found. */
    public AprilTagDetection getTagBySpecificId(int id) {
        if (currentDetections == null) return null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /** Get the first detection that has metadata, or null. */
    public AprilTagDetection getFirstDetection() {
        if (currentDetections == null) return null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    /** Get all current detections. */
    public List<AprilTagDetection> getDetections() {
        return currentDetections;
    }

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

