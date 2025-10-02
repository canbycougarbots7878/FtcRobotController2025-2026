package org.firstinspires.ftc.teamcode;

//I am in the audience. the +y axis is facing twords me. the +x axis is facing the right of me. the tag is farther away from me than the origen is

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@Autonomous(name = "Drive to AprilTag", group = "Movement")
public class AprilTagMovement extends LinearOpMode {

    // Camera offset relative to robot origin (meters)
    private static final double CAMERA_OFFSET_X = 0.18; // forward
    private static final double CAMERA_OFFSET_Y = 0.03; // left

    // Movement speed
    private static final double MOVE_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // --- Initialize robot with OTOS ---
        MovementLib.Robot robot = new MovementLib.Robot(hardwareMap, true);

        // --- Setup AprilTag detection ---
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Init complete. Press start...");
        telemetry.update();
        waitForStart();

        robot.Reverse_Left();

        boolean tagFixed = false;
        AprilTagDetection targetTag = null;

        while (opModeIsActive()) {

            // --- Read OTOS position ---
            SparkFunOTOS.Pose2D pose = robot.otos.getPosition();
            double robotX = pose.x;
            double robotY = pose.y;
            double robotH = pose.h;

            // --- Process AprilTags ---
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!tagFixed && !detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.ftcPose != null) {

                        // Compute camera offset
                        double robotHrad = Math.toRadians(pose.h);
                        double camX = pose.x + (CAMERA_OFFSET_X * Math.cos(robotHrad)
                                - CAMERA_OFFSET_Y * Math.sin(robotHrad));
                        double camY = pose.y + (CAMERA_OFFSET_X * Math.sin(robotHrad)
                                + CAMERA_OFFSET_Y * Math.cos(robotHrad));

                        // Correct robot position using detected tag pose
                        double robotX_corrected = camX - tag.ftcPose.x * 0.0254; // inches → meters
                        double robotY_corrected = camY - tag.ftcPose.y * 0.0254;
                        double robotH_corrected = pose.h - tag.ftcPose.yaw;

                        robot.otos.setPosition(new SparkFunOTOS.Pose2D(
                                robotX_corrected,
                                robotY_corrected,
                                robotH_corrected
                        ));

                        robotX = robotX_corrected;
                        robotY = robotY_corrected;
                        robotH = robotH_corrected;

                        tagFixed = true;
                        targetTag = tag;
                        break;
                    }
                }
            }

            // --- Move toward the detected tag ---
            if (tagFixed && targetTag != null && targetTag.ftcPose != null) {

                double targetX = targetTag.ftcPose.x * 0.0254; // meters
                double targetY = targetTag.ftcPose.y * 0.0254;

                double dx = targetX - robotX;
                double dy = targetY - robotY;

                double distance = Math.hypot(dx, dy);

                double stopDistance = 0.1; // 10 cm tolerance

                if (distance > stopDistance) {
                    double robotHrad = Math.toRadians(robotH);
                    double k = 1.5; // proportional gain
                    double forward = Math.cos(robotHrad) * dy + Math.sin(robotHrad) * dx;
                    double right   = -Math.sin(robotHrad) * dy + Math.cos(robotHrad) * dx;

                    // scale to motor power
                    forward = Math.max(-1.0, Math.min(1.0, forward * k));
                    right   = Math.max(-1.0, Math.min(1.0, right * k));

                    robot.Omni_Move(forward, right, 0, MOVE_SPEED);
                } else {
                    robot.Stop_Wheels();
                    telemetry.addLine("Reached AprilTag!");
                }
            }

            // --- Telemetry ---
            telemetry.addData("Robot Pos (m)", "X=%.2f, Y=%.2f", robotX, robotY);
            telemetry.addData("Heading (deg)", robotH);
            telemetry.addData("Tag fixed?", tagFixed);
            telemetry.update();

            sleep(50);
        }

        visionPortal.close();
    }
}
