package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Disabled
@TeleOp(name = "OTOS Reader", group = "Sensor")
public class OTOS_Reader extends LinearOpMode {
    SparkFunOTOS myOtos;
    MovementLib.Robot robot;

    private AprilTagProcessor aprilTag;

    private double aprilTagXm = 1.50; //m
    private double aprilTagYm = 1.355; //m
    private double aprilTagAngle = 60.5; //degrees

    public SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0);
    public void runOpMode() {
        initAprilTag();

        robot = new MovementLib.Robot(hardwareMap).enableOtos(); // Init Robot with Otos enabled
        robot.Reverse_Left(); // Make all motors spin forward


        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();


        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        waitForStart();
        while(opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            if(gamepad1.start) myOtos.resetTracking();

            AprilTagDetection detection = getFirstDetection();
            if (detection != null && detection.metadata != null) {
                double robotX = 0;
                double robotY = 0;
                double robotHeading = 0;
                if (detection.metadata.id == 20) {
                    double barring = detection.ftcPose.bearing;
                    double yaw = detection.ftcPose.yaw;
                    double range = detection.ftcPose.range;

                    robotHeading = -yaw + aprilTagAngle + 180;
                    double RangeHeading = robotHeading + barring;

                    double RangeHeadingRad = Math.toRadians(RangeHeading);

                    robotX = aprilTagXm - (0.0254 * (range)) * Math.cos(RangeHeadingRad);
                    robotY = aprilTagYm + (0.0254 * (range)) * Math.sin(RangeHeadingRad);

                }
                pos.x = robotX;
                pos.y = robotY;
                pos.h = robotHeading;
                myOtos.setPosition(pos);

            }

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);


            robot.Omni_Move_To_Target(target, 0.5);


            // Update the telemetry on the driver station
            telemetry.update();
        }
    }

    private AprilTagDetection getFirstDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;
        return detections.get(0);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }
}
