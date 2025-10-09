package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTagMovementWithController", group = "Sensor")
public class AprilTagMovementWithControler extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    SparkFunOTOS myOtos;
    MovementLib.Robot robot = null;
    IMU imu;

    FieldTransform fieldTransform;
    double imuStartYaw = 0; // Starting yaw offset

    // Field coordinate transform
    public static class FieldTransform {
        double offsetX; // inches
        double offsetY;
        double offsetH; // degrees

        public FieldTransform(double offsetX, double offsetY, double offsetH) {
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.offsetH = offsetH;
        }

        public SparkFunOTOS.Pose2D toFieldCoords(SparkFunOTOS.Pose2D robotPose) {
            double rad = Math.toRadians(offsetH);
            double cos = Math.cos(rad);
            double sin = Math.sin(rad);

            double fieldX = offsetX + robotPose.x * cos - robotPose.y * sin;
            double fieldY = offsetY + robotPose.x * sin + robotPose.y * cos;
            double fieldH = robotPose.h + offsetH;

            return new SparkFunOTOS.Pose2D(fieldX, fieldY, fieldH);
        }
    }

    @Override
    public void runOpMode() {
        initAprilTag();

        // Hardware initialization
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();

        robot = new MovementLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left);
        robot.Reverse_Right();
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Alliance selection ---
        telemetry.addLine("Press A for Red Alliance, B for Blue Alliance");
        telemetry.update();
        while (!isStopRequested() && !(gamepad1.a || gamepad1.b)) { }

        boolean isRedAlliance = gamepad1.a;
        double sixFeetInInches = 6 * 12.0;
        if (isRedAlliance) {
            fieldTransform = new FieldTransform(sixFeetInInches, sixFeetInInches, 0);
        } else {
            fieldTransform = new FieldTransform(-sixFeetInInches, -sixFeetInInches, 180);
        }

        telemetry.addData("Alliance selected", isRedAlliance ? "Red" : "Blue");
        telemetry.update();

        // Capture starting IMU yaw
        imuStartYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // --- 1. Read joystick input ---
                double localForward = gamepad1.left_stick_y;
                double localRight = -gamepad1.left_stick_x;

                // --- 2. Get current yaw and apply starting offset ---
                double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double yaw = currentYaw - imuStartYaw;

                // --- 3. Alliance multipliers ---
                double xMult = isRedAlliance ? 1.0 : -1.0;
                double yMult = isRedAlliance ? 1.0 : -1.0;

                // --- 4. Transform joystick vector into field-centric global frame ---
                double globalX = xMult * (localForward * Math.cos(yaw) - localRight * Math.sin(yaw));
                double globalY = yMult * (localForward * Math.sin(yaw) + localRight * Math.cos(yaw));

                // --- 5. Drive robot ---
                robot.Omni_Move(globalX, globalY, gamepad1.right_stick_x,
                        (gamepad1.right_bumper ? 1.0 : 0.5));

                // --- 6. Reset heading if needed ---
                if (gamepad1.start) imu.resetYaw();

                // --- 7. Update field-aligned OTOS pose ---
                SparkFunOTOS.Pose2D localPose = myOtos.getPosition();
                SparkFunOTOS.Pose2D fieldPose = fieldTransform.toFieldCoords(localPose);

                // --- 8. Update telemetry ---
                telemetryAprilTag();

                telemetry.addLine("\n--- Robot Field Position ---");
                telemetry.addData("Field X (in)", fieldPose.x);
                telemetry.addData("Field Y (in)", fieldPose.y);
                telemetry.addData("Field Heading (deg)", fieldPose.h);
                telemetry.addData("Yaw (deg)", Math.toDegrees(yaw));
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

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        final double FT = 12.0;
        class TagFieldPose {
            double x, y, h;
            TagFieldPose(double x, double y, double h) { this.x = x; this.y = y; this.h = h; }
        }

        TagFieldPose tag20 = new TagFieldPose(-4.75040454424 * FT, -2.07921462701 * FT, 53.3092886396);  // Blue tag
        TagFieldPose tag24 = new TagFieldPose(-4.75040454424 * FT, 2.07921462701 * FT, -53.3092886396);  // Red tag

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                SparkFunOTOS.Pose2D cameraPose = new SparkFunOTOS.Pose2D(detection.ftcPose.y, -detection.ftcPose.x, detection.ftcPose.yaw);
                myOtos.setPosition(cameraPose);
                pos = myOtos.getPosition();

                // Auto-correct robot field pose using tag
                TagFieldPose knownTag = (detection.id == 20) ? tag20 : (detection.id == 24) ? tag24 : null;
                if (knownTag != null) {
                    double correctedX = knownTag.x - pos.x;
                    double correctedY = knownTag.y - pos.y;
                    double correctedH = knownTag.h - pos.h;
                    SparkFunOTOS.Pose2D correctedPose = new SparkFunOTOS.Pose2D(correctedX, correctedY, correctedH);
                    myOtos.setPosition(correctedPose);
                    pos = myOtos.getPosition();
                    telemetry.addLine(String.format("Corrected using Tag ID %d", detection.id));
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}
