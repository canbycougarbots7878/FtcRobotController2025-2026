package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="aiming Auto", group="Robot")
public class aimingAuto extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    SparkFunOTOS myOtos;
    MovementLib.Robot robot = null;

    public void runOpMode() {
        initAprilTag();

        // Hardware initialization
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        robot = new MovementLib.Robot(hardwareMap);

        robot.Reverse_Left();

        waitForStart();

        if (opModeIsActive()) {


            robot.Omni_Move( 0.5, 0, 0, 1.0);

            sleep(1500);

            robot.Omni_Move( 0, 0, 0, 0.0);

            while (opModeIsActive()) {
                AprilTagDetection detection = getFirstDetection();
                if (detection != null && detection.metadata != null) {
                    double barring = detection.ftcPose.bearing;
                    double yaw = detection.ftcPose.yaw;
                    double Ydistance = detection.ftcPose.y;
                    double distanceFromYTarget = Ydistance-70;

                    if ((Math.abs(barring) > 0.5) || (Math.abs(yaw) > 5) || (Math.abs(distanceFromYTarget) > 0.5)) {
                        robot.Omni_Move((distanceFromYTarget)/18, (yaw)/18, (barring)/18, 0.5);
                    }

                    telemetry.addData("Barring", barring);
                    telemetry.addData("yaw", yaw);

                    telemetry.update();
                }
                robot.Omni_Move(0, 0, 0, 0);
            }

        }
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
}
