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
    @Override
    public void runOpMode() {
        initAprilTag();
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();


        robot = new MovementLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left); // Initialize Wheels handler
        robot.Reverse_Left(); // Make all motors spin forward
        imu = hardwareMap.get(IMU.class, "imu");


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Get local directions
                double localForward = gamepad1.left_stick_y;
                double localRight = - gamepad1.left_stick_x;


                // Read IMU data
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                double yaw = angles.getYaw() * 0.01745329251; // Get robot yaw converted to radians


                // Calculate global directions using 2D rotation matrix
                double forward = localForward * -Math.cos(yaw) - localRight * Math.sin(yaw);
                double right = localForward * -Math.sin(yaw) + localRight * Math.cos(yaw);


                robot.Omni_Move(forward, right, gamepad1.right_stick_x, (gamepad1.right_bumper ? 1.0 : 0.5));


                if(gamepad1.start) imu.resetYaw();

                telemetryAprilTag();


                telemetry.update();


                // Share the CPU.
                sleep(20);
            }
        }


        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();


    }   // end method runOpMode()


    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();


        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }   // end method initAprilTag()


    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(detection.ftcPose.y, detection.ftcPose.x, detection.ftcPose.yaw);
                myOtos.setPosition(currentPosition);
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                telemetry.addLine();
                // Log the position to the telemetry
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


}   // end class
