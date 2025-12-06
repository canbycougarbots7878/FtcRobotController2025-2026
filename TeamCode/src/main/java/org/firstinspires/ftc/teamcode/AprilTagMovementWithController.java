package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name = "AprilTagMovementWithController", group = "Sensor")
public class AprilTagMovementWithController extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    SparkFunOTOS myOtos;
    RobotLib.Robot robot = null;

    // Unit conversion constants
    private static final double INCH_TO_M = 0.0254;
    private static final double CM_TO_M = 0.01;

    // Camera constants
    private static final double CAMERA_X_CM = 6.5;
    private static final double CAMERA_Y_CM = 4.0;
    private static final double CAMERA_Z_CM = 13.75;

    // Filtering constants
    private static final double MAX_POSITION_JUMP_M = 0.3;
    private static final double MAX_HEADING_JUMP_DEG = 20.0;

    private SparkFunOTOS.Pose2D lastValidPose = null;

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

        robot = new RobotLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left);
        robot.Reverse_Left();

        myOtos.calibrateImu();

        // Alliance selection
        telemetry.addLine("Press A for Red Alliance, B for Blue Alliance");
        telemetry.update();
        while (!isStopRequested() && !(gamepad1.a || gamepad1.b)) {
        }
        boolean isRedAlliance = gamepad1.a;
        telemetry.addData("Alliance selected", isRedAlliance ? "Red" : "Blue");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // --- AprilTag detection ---
                AprilTagDetection detection = getFirstDetection();
                SparkFunOTOS.Pose2D candidatePosition = null;

                if (detection != null && detection.metadata != null) {
                    double tagRange_m = detection.ftcPose.range * INCH_TO_M;
                    double bearing_deg = detection.ftcPose.bearing;
                    double yaw_deg = detection.ftcPose.yaw;
                    int detId = detection.id;

                    // Compute candidate position
                    double deltaZ_m = 29.50 * INCH_TO_M - CAMERA_Z_CM * CM_TO_M;
                    if ((detId == 20 || detId == 24) && tagRange_m > Math.abs(deltaZ_m)) {
                        double XYPlaneRange_m = Math.sqrt(Math.max(0.0, tagRange_m * tagRange_m - deltaZ_m * deltaZ_m));

                        double blueAngleRad = Math.acos((Math.pow(58.7, 2) - Math.pow(70, 2) - Math.pow(59, 2)) / (-2 * 70 * 59));
                        double blueAngleDeg = Math.toDegrees(blueAngleRad);
                        double redAngleDeg = normalize360(360.0 - blueAngleDeg);

                        double BlueX_m = (-72.0 + 35.0 * Math.sin(blueAngleRad)) * INCH_TO_M;
                        double BlueY_m = (-72.0 + (59.0 - 35.0 * Math.cos(blueAngleRad))) * INCH_TO_M;
                        double RedX_m = (-72.0 + 35.0 * Math.sin(Math.toRadians(redAngleDeg))) * INCH_TO_M;
                        double RedY_m = (72.0 - (59.0 - 35.0 * Math.cos(Math.toRadians(redAngleDeg)))) * INCH_TO_M;

                        double tagBaseAngleDeg = (detId == 20) ? blueAngleDeg : redAngleDeg;

                        double cameraAngleDeg = (detId == 20)
                                ? normalize360(yaw_deg + 180.0 + tagBaseAngleDeg)
                                : normalize360(-yaw_deg + 180.0 + tagBaseAngleDeg);

                        double bearingFromXaxisDeg = normalize180(360.0 - (cameraAngleDeg + bearing_deg));
                        double bearingFromXaxisRad = Math.toRadians(bearingFromXaxisDeg);

                        double deltaX_m = XYPlaneRange_m * Math.cos(bearingFromXaxisRad);
                        double deltaY_m = XYPlaneRange_m * Math.sin(bearingFromXaxisRad);

                        double cameraPosX_m = (detId == 20 ? BlueX_m : RedX_m) - deltaX_m;
                        double cameraPosY_m = (detId == 20 ? BlueY_m : RedY_m) - deltaY_m;

                        double cameraAngleRad = Math.toRadians(cameraAngleDeg);
                        double otosX_m = cameraPosX_m + CAMERA_X_CM * CM_TO_M * Math.cos(cameraAngleRad);
                        double otosY_m = cameraPosY_m + CAMERA_Y_CM * CM_TO_M * Math.sin(cameraAngleRad);

                        double otosHeadingDeg = -OTOSNormalize(cameraAngleDeg);

                        candidatePosition = new SparkFunOTOS.Pose2D(otosX_m, otosY_m, otosHeadingDeg);
                    }
                }

                // AprilTag filtering
                if (candidatePosition != null) {
                    boolean validUpdate = true;
                    if (lastValidPose != null) {
                        double dx = candidatePosition.x - lastValidPose.x;
                        double dy = candidatePosition.y - lastValidPose.y;
                        double dist = Math.hypot(dx, dy);
                        double dheading = Math.abs(normalize180(candidatePosition.h - lastValidPose.h));
                        if (dist > MAX_POSITION_JUMP_M || dheading > MAX_HEADING_JUMP_DEG) {
                            validUpdate = false;
                        }
                    }
                    if (validUpdate) {
                        myOtos.setPosition(candidatePosition);
                        lastValidPose = candidatePosition;
                        telemetry.addLine("OTOS updated from AprilTag");
                    }
                }

                // --- Driver control / move-to-origin ---
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();

                if (gamepad1.y) {
                    // Move to origin (0,0) heading 0°
                    SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0, 0, 0);
                    double dx = target.x - pos.x;
                    double dy = target.y - pos.y;
                    double dh = normalize180(target.h - pos.h);

                    double forward = Math.max(-1.0, Math.min(1.0, dx * 2));
                    double right = Math.max(-1.0, Math.min(1.0, dy * 2));
                    double rotate = Math.max(-1.0, Math.min(1.0, dh / 90.0));

                    robot.Omni_Move(forward, right, rotate, 0.5);
                    telemetry.addLine(String.format("Moving to origin: dx=%.2f dy=%.2f dh=%.1f", dx, dy, dh));
                } else {
                    // Field-centric driver control
                    double radians = Math.toRadians(pos.h);
                    double robotForward = gamepad1.left_stick_y;
                    double robotRight = gamepad1.left_stick_x;

                    double Forward = robotForward * Math.cos(radians) - robotRight * Math.sin(radians);
                    double Right = robotForward * Math.sin(radians) + robotRight * Math.cos(radians);

                    robot.Omni_Move(-Forward, -Right, -gamepad1.right_stick_x,
                            (gamepad1.right_bumper ? 1.0 : 0.5));
                }

                telemetry.addLine(String.format("XYH (m,m,deg): %6.3f %6.3f %6.1f", pos.x, pos.y, pos.h));
                telemetry.update();
                sleep(50);
            }
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    private AprilTagDetection getFirstDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;
        return detections.get(0);
    }

    private double normalize180(double angle) {
        angle = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return angle;
    }

    private double normalize360(double angle) {
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double OTOSNormalize(double Angle) {
        return normalize180(Angle);
    }
}
