package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetector {
    public VisionPortal vision_portal;
    public AprilTagProcessor april_tag_processor;
    public List<AprilTagDetection> current_detections;
    public ElapsedTime last_check;

    public AprilTagDetector(HardwareMap hardwareMap) {
        this.april_tag_processor = AprilTagProcessor.easyCreateWithDefaults();
        this.vision_portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), april_tag_processor);
        this.current_detections = april_tag_processor.getDetections();
        this.last_check = new ElapsedTime();
    }

    /**
     * Print april tag details through telemetry
     */
    public void telemetryAprilTags(Telemetry telemetry) {
        for (AprilTagDetection detection : current_detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ (Raw) %6.1f %6.1f %6.1f", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine("\n---- Magic");
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            }
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

    /**
     * Detect new apriltags and update memory bank
     */
    public void detect() {
        this.current_detections = april_tag_processor.getDetections(); // Update tags in memory
        this.last_check.reset(); // Reset timer
    }

    /**
     * Get apriltag instance from memory bank
     */
    public AprilTagDetection findByID(int id) {
        for (AprilTagDetection detection : current_detections) {
            if (detection.id == id) return detection;
        }
        return null; // Couldn't find
    }
}
