package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Aiming Test", group="Robot")
public class aimingCode extends LinearOpMode {
    private final double SPINNER_VELOCITY = 1150;
    private final int ARM_POS = 640;

    public DcMotorEx Arm = null;
    private DcMotorEx leftSpinner, rightSpinner;
    private Servo pushServo = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    SparkFunOTOS myOtos;
    MovementLib.Robot robot = null;

    int stage = 1;

    public void runOpMode() {
        initAprilTag();

        robot = new MovementLib.Robot(hardwareMap).enableArm();

        robot.Reverse_Left();

        Arm = hardwareMap.get(DcMotorEx.class, "arm");
        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        leftSpinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSpinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        pushServo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        if (opModeIsActive()) {
            boolean targetPoseAchieved = false;
            double RobotTurn = 0.1;
            int rotateCounter = 0;
            int rotateCounterLimit = 100;

            while (opModeIsActive()) {
                AprilTagDetection detection = getFirstDetection();
                if (detection != null && detection.metadata != null) {
                    if (detection.metadata.id == 20){
                        double barring = detection.ftcPose.bearing;
                        double yaw = detection.ftcPose.yaw;
                        double Ydistance = detection.ftcPose.y;
                        double TargetYDis = 70;
                        double YDisDif = TargetYDis - Ydistance;
                        RobotTurn = 0.1;

                        if (((Math.abs(barring) > 1)) && !targetPoseAchieved) {
                            robot.Omni_Move(0, 0, (barring)/18, 1);
                        } else if (((Math.abs(barring) > 1)) && targetPoseAchieved) {
                            targetPoseAchieved = false;
                        } else if (!targetPoseAchieved) {
                            targetPoseAchieved = true;
                            robot.Omni_Move(0, 0, 0, 0);
                        }else {
                            telemetry.addLine(":)");
                            LaunchBall();
                        }

                        telemetry.addData("Barring", barring);
                        telemetry.addData("yaw", yaw);
                        telemetry.addData("distance From apriltag", Ydistance);

                        telemetry.update();
                    }

                }else {
                    robot.Omni_Move(0, 0, RobotTurn, 1);
                    if (rotateCounter >= rotateCounterLimit){
                        RobotTurn += ((Math.abs(RobotTurn)+0.1)*(-Math.copySign(1, RobotTurn)));
                        rotateCounter = -1;
                        rotateCounterLimit *= 2;
                    }
                    rotateCounter += 1;
                }
                if(stage > 0) {
                    LaunchBall();
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

    private boolean LaunchBall() {
        if(Arm.getCurrentPosition() > ARM_POS - 10 && stage == 1) {
            stage = 2;
        }
        if(leftSpinner.getVelocity() > SPINNER_VELOCITY - 10 && stage == 2) {
            stage = 3;
        }
        if(leftSpinner.getVelocity() > SPINNER_VELOCITY - 10 && stage == 3) {
            stage = 4;
        }
        switch(stage) {
            case 1:
                Arm.setPower(1);
                robot.Arm_Motor.setTargetPosition(ARM_POS); // Move arm up
                break;
            case 2:
                leftSpinner.setVelocity(SPINNER_VELOCITY);
                rightSpinner.setVelocity(SPINNER_VELOCITY);
                break;
            case 3:
                pushServo.setPosition(0.7);
                break;
        }
        return true;
    }
}
