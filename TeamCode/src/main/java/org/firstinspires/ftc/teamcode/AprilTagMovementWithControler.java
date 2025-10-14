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
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
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

    double imuStartYaw = 0; // Starting yaw offset

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
        robot.Reverse_Right();

        double BlueAllianceTagH = Math.acos((Math.pow(58.7, 2) - Math.pow(70, 2) - Math.pow(59, 2))/(-2*70*59));
        double BlueAllianceTagXposin = -72 + (35*(0.39370079))*Math.sin(BlueAllianceTagH);
        double BlueAllianceTagYposin = -72 + (59-(35)*Math.cos(BlueAllianceTagH))*(0.39370079);

        double RedAllianceTagH = 360 - Math.acos((Math.pow(58.7, 2) - Math.pow(70, 2) - Math.pow(59, 2))/(-2*70*59));
        double RedAllianceTagXposin = -72 + (35*(0.39370079))*Math.sin(RedAllianceTagH);
        double RedAllianceTagYposin = 72 - (59-(35)*Math.cos(RedAllianceTagH))*(0.39370079);

        // --- Alliance selection ---
        telemetry.addLine("Press A for Red Alliance, B for Blue Alliance");
        telemetry.update();
        while (!isStopRequested() && !(gamepad1.a || gamepad1.b)) {
        }

        boolean isRedAlliance = gamepad1.a;

        telemetry.addData("Alliance selected", isRedAlliance ? "Red" : "Blue");
        telemetry.update();

        if (isRedAlliance) {
            String Alliance = "Red";
        } else {
            String Alliance = "Blue";
        }


        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                telemetryAprilTag();



                SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
                myOtos.setPosition(currentPosition);
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                double robotForward = gamepad1.left_stick_y;
                double robotRight = gamepad1.left_stick_x;
                double Radians = Math.toRadians(pos.h);

                double Forward = robotForward*Math.cos(Radians) - robotRight*Math.sin(Radians);
                double Right = -robotRight*Math.cos(Radians) + robotForward*Math.sin(Radians);

                robot.Omni_Move( Forward, Right, -gamepad1.right_stick_x, (gamepad1.right_bumper ? 1.0 : 0.5));

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

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                double TagX = detection.ftcPose.x;
                double TagY = detection.ftcPose.y;
                double TagYaw = detection.ftcPose.yaw;

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            telemetry.update();
        }
    }
}
