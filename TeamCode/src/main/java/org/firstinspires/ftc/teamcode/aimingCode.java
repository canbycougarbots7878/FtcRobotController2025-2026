package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Aiming", group = "Sensor")
public class aimingCode extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    SparkFunOTOS myOtos;
    MovementLib.Robot robot = null;

    public void runOpMode(){
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
            while (opModeIsActive()) {
                AprilTagDetection detection = getFirstDetection();

                robot.Omni_Move(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, (gamepad1.right_bumper ? 1.0 : 0.5));

                if (detection != null && detection.metadata != null) {
                    double barring = detection.ftcPose.bearing;
                    if ((Math.abs(barring) > 1) && gamepad1.start){
                        robot.Omni_Move(-gamepad1.left_stick_y, gamepad1.left_stick_x, (barring)/18, (gamepad1.right_bumper ? 1.0 : 0.5));
                    }
                }

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
