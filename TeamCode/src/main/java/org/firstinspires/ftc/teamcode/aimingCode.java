package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Math.DegTrig;
import org.firstinspires.ftc.teamcode.Math.unitConversion;

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
    RobotLib.Robot robot = null;

    int stage = 0;
    boolean targetPoseAchieved = false;
    double RobotTurn = 0.1;
    int rotateCounter = 0;
    int rotateCounterLimit = 100;

    double barring;
    double yaw;
    double Ydistance;
    double TargetYDis;
    double YDisDif;
    double robotX = 0;
    double robotY = 0;
    double robotHeading = 0;
    public SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(1.21,0,-135);

    public void runOpMode() {
        initAprilTag();

        robot = new RobotLib.Robot(hardwareMap).enableArm().enableOtos();

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


            while (opModeIsActive()) {
                if (stage < 6){
                    robotAiming();
                }
                if(stage >= 6) {
                    LaunchBall();
                }
                telemetry.addData("Stage",stage);
                telemetry.update();
            }

        }
    }
    private boolean hello() {
        double aprilTagXm = -1.50; //m
        double aprilTagYm = -1.355; //m
        double aprilTagAngle = 60.5; //degrees
        SparkFunOTOS.Pose2D pos = robot.otos.getPosition();
        if(gamepad1.start) robot.otos.resetTracking();

        AprilTagDetection detection = getFirstDetection();
        if (detection != null && detection.metadata != null) {

            if (detection.metadata.id == 20) {
                double barring = detection.ftcPose.bearing;
                double yaw = detection.ftcPose.yaw;
                double range = unitConversion.inTom(detection.ftcPose.range);

                robotHeading = -yaw + aprilTagAngle + 180;
                double RangeHeading = robotHeading + barring;

                double CameraHeading = -160.5 + RangeHeading;

                robotX = aprilTagXm - (range) * DegTrig.cosDeg(RangeHeading) + (0.195) * DegTrig.cosDeg(CameraHeading) + 0.415;

                if (Math.abs(robotX) > unitConversion.inTom(72)){
                    robotX = aprilTagXm + (range) * DegTrig.cosDeg(RangeHeading) + (0.195) * DegTrig.cosDeg(CameraHeading) + 0.415;
                }

                double mathTest ;

                robotY = aprilTagYm - (range) * DegTrig.sinDeg(RangeHeading) + (0.195) * DegTrig.sinDeg(CameraHeading) - 0.255;

                if (Math.abs(robotY) > unitConversion.inTom(72)){
                    robotY = aprilTagYm + (range) * DegTrig.sinDeg(RangeHeading) + (0.195) * DegTrig.sinDeg(CameraHeading) - 0.255;
                }

                pos.x = robotX;
                pos.y = robotY;
                pos.h = robotHeading;
                robot.otos.setPosition(pos);

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
            return true;
        }
        return false;
    }
    private void robotAiming(){
        AprilTagDetection detection = getFirstDetection();
        if (detection != null && detection.metadata != null && stage == 1) {
            stage = 2;
        } else if (stage == 2){
            stage = 1;
        }

        switch (stage){
            case 0:
                if(hello()) {
                    stage = 1;
                }
                break;
            case 1:
                robot.Omni_Move(0, 0, RobotTurn, 1);
                if (rotateCounter >= rotateCounterLimit){
                    RobotTurn += ((Math.abs(RobotTurn)+0.1)*(-Math.copySign(1, RobotTurn)));
                    rotateCounter = -1;
                    rotateCounterLimit *= 2;
                }
                rotateCounter += 1;
                break;
            case 2:
                if (detection.metadata.id == 20){
                    stage = 3;
                }
                break;
            case 3:
                assert detection != null;
                barring = detection.ftcPose.bearing;
                yaw = detection.ftcPose.yaw;
                Ydistance = detection.ftcPose.y;
                TargetYDis = 70;
                YDisDif = TargetYDis - Ydistance;

                RobotTurn = 0.1;

                if (((Math.abs(barring) > 1))) {
                    stage = 4;
                }else {
                    stage = 5;
                }
                break;
            case 4:
                robot.Omni_Move(0, 0, (barring)/20, 1);
                stage = 0;
                break;
            case 5:
                telemetry.addLine(":)");
                stage = 6;
                robot.Omni_Move(0, 0, 0, 0);
                LaunchBall();
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

    private void LaunchBall() {
        if(Arm.getCurrentPosition() > ARM_POS - 10 && stage == 6) {
            stage = 7;
        }
        if(leftSpinner.getVelocity() > SPINNER_VELOCITY - 10 && stage == 7) {
            stage = 8;
        }
        switch(stage) {
            case 6:
                Arm.setPower(1);
                robot.Arm_Motor.setTargetPosition(ARM_POS); // Move arm up
                telemetry.addData("Arm going up", robot.Arm_Motor.getTargetPosition());
                break;
            case 7:
                leftSpinner.setVelocity(SPINNER_VELOCITY);
                rightSpinner.setVelocity(SPINNER_VELOCITY);
                telemetry.addData("Speed", leftSpinner.getVelocity());
                telemetry.addData("Target Speed", leftSpinner.getVelocity());
                break;
            case 8:
                pushServo.setPosition(0.7);
                break;
        }
    }
}
