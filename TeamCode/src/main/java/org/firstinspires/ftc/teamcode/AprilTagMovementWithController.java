package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTagMovementWithController", group = "Sensor")
public class AprilTagMovementWithController extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    SparkFunOTOS myOtos;
    MovementLib.Robot robot = null;

    // Unit conversion constants
    private static final double INCH_TO_M = 0.0254;
    private static final double CM_TO_M = 0.01;

    // Field / camera constants (edit these if your hardware offsets change)
    private static final double CAMERA_X_CM = 6.5; // camera offset from OTOS in centimeters (X)
    private static final double CAMERA_Y_CM = 4.0; // camera offset from OTOS in centimeters (Y)
    private static final double CAMERA_Z_CM = 13.75; // camera height above floor in cm

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        initAprilTag();

        // Hardware initialization
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.resetTracking();

        robot = new MovementLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left);
        robot.Reverse_Left();

        // Precompute tag geometry using consistent units (meters)
        // Law of cosines angle (radians) for your field geometry constants (unchanged numeric intent)
        double blueAngleRad = Math.acos((Math.pow(58.7, 2) - Math.pow(70, 2) - Math.pow(59, 2)) / (-2 * 70 * 59));
        double blueAngleDeg = Math.toDegrees(blueAngleRad);
        double redAngleDeg = normalize360(360.0 - blueAngleDeg);
        double redAngleRad = Math.toRadians(redAngleDeg);

        // Convert field references (original values looked like inches) into meters
        // These are the tag positions on the field in meters
        double BlueAllianceTagX_m = (-72.0 + 35.0 * Math.sin(blueAngleRad)) * INCH_TO_M;
        double BlueAllianceTagY_m = (-72.0 + (59.0 - 35.0 * Math.cos(blueAngleRad))) * INCH_TO_M;
        double BlueAllianceTagZ_m = 29.50 * INCH_TO_M;

        double RedAllianceTagX_m = (-72.0 + 35.0 * Math.sin(redAngleRad)) * INCH_TO_M; // mirrored X, adjust as needed
        double RedAllianceTagY_m = (72.0 - (59.0 - 35.0 * Math.cos(redAngleRad))) * INCH_TO_M; // mirrored Y, adjust as needed

        // Camera/OTOS offsets in meters
        final double cameraXdistanceFromOTOS_m = CAMERA_X_CM * CM_TO_M;
        final double cameraYdistanceFromOTOS_m = CAMERA_Y_CM * CM_TO_M;
        final double cameraZPos_m = CAMERA_Z_CM * CM_TO_M;

        // Vertical delta between tag Z and camera Z (meters)
        final double DeltaZ_m = BlueAllianceTagZ_m - cameraZPos_m;

        // --- Alliance selection ---
        telemetry.addLine("Press A for Red Alliance, B for Blue Alliance");
        telemetry.update();
        while (!isStopRequested() && !(gamepad1.a || gamepad1.b)) {
            // waiting for selection
        }

        boolean isRedAlliance = gamepad1.a;
        String alliance = isRedAlliance ? "Red" : "Blue";

        telemetry.addData("Alliance selected", alliance);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the single best detection (or null)
                AprilTagDetection detection = getFirstDetection();
                boolean detectionValid = false;
                int detId = -1;
                double tagRange_m = 0.0;
                double bearing_deg = 0.0;
                double yaw_deg = 0.0;

                if (detection != null && detection.metadata != null) {
                    // NOTE: SDK seems to provide ftcPose.range in inches in your previous telemetry.
                    // Convert to meters here. If your SDK already gives meters, remove conversion.
                    tagRange_m = detection.ftcPose.range * INCH_TO_M;
                    bearing_deg = detection.ftcPose.bearing;
                    yaw_deg = detection.ftcPose.yaw;
                    detId = detection.id;
                    detectionValid = true;

                    telemetry.addLine(String.format("Detection ID %d  range(in)=%.2f  bearing=%.2f  yaw=%.2f",
                            detection.id, detection.ftcPose.range, bearing_deg, yaw_deg));
                } else {
                    telemetry.addLine("No AprilTag detections");
                }

                SparkFunOTOS.Pose2D candidatePosition = null;

                // Only attempt fusion when we have a valid detection and sane geometry
                if (detectionValid && (detId == 20 || detId == 24) && tagRange_m > Math.abs(DeltaZ_m)) {
                    // Compute XY-plane range from range and vertical offset
                    double XYPlaneRange_m = Math.sqrt(Math.max(0.0, tagRange_m * tagRange_m - DeltaZ_m * DeltaZ_m));

                    // Select which tag position & base heading to use
                    double tagX_m = (detId == 20) ? BlueAllianceTagX_m : RedAllianceTagX_m;
                    double tagY_m = (detId == 20) ? BlueAllianceTagY_m : RedAllianceTagY_m;
                    double tagBaseAngleDeg = (detId == 20) ? blueAngleDeg : redAngleDeg;

                    // Compute camera (heading) angle in degrees.
                    // The formulas follow your original intent but use normalized angles.
                    double cameraAngleDeg;
                    if (detId == 20) {
                        cameraAngleDeg = normalize360(yaw_deg + 180.0 + tagBaseAngleDeg);
                    } else { // detId == 24
                        cameraAngleDeg = normalize360(-yaw_deg + 180.0 + tagBaseAngleDeg);
                    }

                    // Bearing from X axis (signed) in degrees; normalize to [-180,180)
                    double bearingFromXaxisDeg = normalize180(360.0 - (cameraAngleDeg + bearing_deg));
                    double bearingFromXaxisRad = Math.toRadians(bearingFromXaxisDeg);

                    // Delta X/Y in meters (field coordinates)
                    double deltaX_m = XYPlaneRange_m * Math.cos(bearingFromXaxisRad);
                    double deltaY_m = XYPlaneRange_m * Math.sin(bearingFromXaxisRad);

                    // Camera position in field frame (meters)
                    double cameraPosX_m = tagX_m - deltaX_m;
                    double cameraPosY_m = tagY_m - deltaY_m;

                    // OTOS position = camera position + vector from camera to OTOS rotated by camera heading
                    double cameraAngleRad = Math.toRadians(cameraAngleDeg);
                    double otosX_m = cameraPosX_m + cameraXdistanceFromOTOS_m * Math.cos(cameraAngleRad);
                    double otosY_m = cameraPosY_m + cameraYdistanceFromOTOS_m * Math.sin(cameraAngleRad);

                    // OTOS heading normalized to -180..180 for SparkFunOTOS
                    double otosHeadingDeg = OTOSNormalize(cameraAngleDeg);

                    candidatePosition = new SparkFunOTOS.Pose2D(otosX_m, otosY_m, otosHeadingDeg);

                    telemetry.addLine(String.format("Tag XY-plane range (m): %.3f  deltaZ(m): %.3f", XYPlaneRange_m, DeltaZ_m));
                    telemetry.addLine(String.format("Camera pos (m): %.3f, %.3f  OTOS pos (m): %.3f, %.3f  h: %.1f",
                            cameraPosX_m, cameraPosY_m, otosX_m, otosY_m, otosHeadingDeg));
                }

                // Apply candidate position only when valid (simple sanity gating)
                if (candidatePosition != null) {
                    myOtos.setPosition(candidatePosition);
                    telemetry.addLine("OTOS updated from AprilTag");
                } else {
                    telemetry.addLine("OTOS not updated (no valid candidate)");
                }

                // Read OTOS pose for driver-relative control
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();

                // Driver controls (field-centric)
                double robotForward = gamepad1.left_stick_y;
                double robotRight = gamepad1.left_stick_x;
                double radians = Math.toRadians(pos.h);

                // Rotate joystick vector by robot heading to get field-centric commands
                double Forward = robotForward * Math.cos(radians) - robotRight * Math.sin(radians);
                double Right = -robotRight * Math.cos(radians) + robotForward * Math.sin(radians);

                robot.Omni_Move(-Forward, -Right, -gamepad1.right_stick_x, (gamepad1.right_bumper ? 1.0 : 0.5));

                telemetry.addLine(String.format("XYH (m,m,deg): %6.3f %6.3f %6.1f", pos.x, pos.y, pos.h));
                telemetry.update();

                sleep(20);
            }
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    // Return the first detection (or null). This keeps detection polling simple and avoids multiple repeated scans.
    private AprilTagDetection getFirstDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;
        return detections.get(0);
    }

    // Normalize into [-180, 180)
    private double normalize180(double angle) {
        angle = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return angle;
    }

    // Normalize into [0,360)
    private double normalize360(double angle) {
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    // Convert heading to OTOS-friendly [-180,180) representation (keeps sign)
    private double OTOSNormalize(double Angle) {
        return normalize180(Angle);
    }

    // Removed previous telemetryAprilTag multi-call helper; we use getFirstDetection() instead.
}
