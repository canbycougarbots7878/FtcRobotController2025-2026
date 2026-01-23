package org.firstinspires.ftc.teamcode.TestingCode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libraries.RobotLib;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Disabled
@TeleOp(name = "OTOS Reader", group = "Sensor")
public class OTOS_Reader extends LinearOpMode {
    SparkFunOTOS myOtos;
    RobotLib.Robot robot;

    private AprilTagProcessor aprilTag;

    private double aprilTagXm = -1.50; //m
    private double aprilTagYm = -1.355; //m
    private double aprilTagAngle = 60.5; //degrees

    public SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(1.21,0,-135);
    public void runOpMode() {
        initAprilTag();

        robot = new RobotLib.Robot(hardwareMap).enableOtos(); // Init Robot with Otos enabled
        robot.Reverse_Left(); // Make all motors spin forward


        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();


        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        double robotX = 0;
        double robotY = 0;
        double robotHeading = 0;

        waitForStart();
        while(opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            if(gamepad1.start) myOtos.resetTracking();

            AprilTagDetection detection = getFirstDetection();
            if (detection != null && detection.metadata != null) {

                if (detection.metadata.id == 20) {
                    double barring = detection.ftcPose.bearing;
                    double yaw = detection.ftcPose.yaw;
                    double range = detection.ftcPose.range * 0.0254;

                    robotHeading = -yaw + aprilTagAngle + 180;
                    double RangeHeading = robotHeading + barring;

                    double CameraHeading = -160.5 + RangeHeading;

                    double RangeHeadingRad = Math.toRadians(RangeHeading);

                    double CameraHeadingRad = Math.toRadians(CameraHeading);

                    robotX = aprilTagXm - (range) * Math.cos(RangeHeadingRad) + (0.195) * Math.cos(CameraHeadingRad) + 0.415;

                    if (Math.abs(robotX) > 1.8288){
                        robotX = aprilTagXm + (range) * Math.cos(RangeHeadingRad) + (0.195) * Math.cos(CameraHeadingRad) + 0.415;
                    }

                    robotY = aprilTagYm - (range) * Math.sin(RangeHeadingRad) + (0.195) * Math.sin(CameraHeadingRad) - 0.255;

                    if (Math.abs(robotY) > 1.8288){
                        robotY = aprilTagYm - (range) * Math.sin(RangeHeadingRad) + (0.195) * Math.sin(CameraHeadingRad) - 0.255;
                    }

                    pos.x = robotX;
                    pos.y = robotY;
                    pos.h = robotHeading;
                    myOtos.setPosition(pos);

                    telemetry.addData("barring", barring);
                    telemetry.addData("yaw", yaw);
                    telemetry.addData("range", range);
                    telemetry.addLine();
                    telemetry.addData("robotHeading", robotHeading);
                    telemetry.addData("RangeHeading", RangeHeading);
                    telemetry.addData("CameraHeading", CameraHeading);
                    telemetry.addLine();
                    telemetry.addData("robotX", robotX);
                    telemetry.addData("robotY", robotY);
                    telemetry.addLine();
                }
            }

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);


            robot.Omni_Move_To_Target(target, 0.5);


            if (robot.Distance_To(target) < 0.5){
                telemetry.addLine();
                telemetry.addLine(";)");
            }


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
