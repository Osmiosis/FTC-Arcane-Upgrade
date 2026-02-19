package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.List;

/**
 * Localization stub - replaces the AprilTag/Webcam-based Localization implementation
 * with a no-op stub so the project compiles without camera dependencies.
 *
 * Methods preserve the original public API but return safe defaults.
 */
public class Localization {

    private final Telemetry telemetry;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // No-op: camera / apriltag initialization removed intentionally
        if (this.telemetry != null) {
            this.telemetry.addData("Localization", "Stub initialized (camera removed)");
            this.telemetry.update();
        }
    }

    public void setManualExposure() {
        // No-op
    }

    /**
     * Old method drove to an AprilTag; stub returns false indicating no target found.
     */
    public boolean driveToAprilTag(int targetId, Object drive) {
        // No-op: indicate target not found
        return false;
    }

    public boolean isTagVisible(int targetId) {
        return false;
    }

    public List<Object> getAllDetections() {
        return Collections.emptyList();
    }

    public Object getDesiredTag() {
        return null;
    }

    public void close() {
        // No-op
    }
}
