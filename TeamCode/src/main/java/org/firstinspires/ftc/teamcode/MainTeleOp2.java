package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.Reader;
import java.util.Scanner;

@TeleOp(name = "Main TeleOp 2", group = "Official")
public class MainTeleOp2 extends LinearOpMode {
    DriveBase driveBase = null;
    Arm arm = null;
    Shooter shooter = null;
    int target_apriltag = 20; // Blue team
    Boolean PRM_ENABLED = false;
    Boolean arm_up = false;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.back) {
                // Config mode
                handleConfig();
            }
            else {
                // Main operation
                handleDrive();
                handleHoming();
                if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                    arm_up = !arm_up;
                }
                handleShooter();
                handleArm();
            }
            updateTelemetry();
        }
    }

    // ------------------ Initialization ------------------ //
    private void initHardware() {
        driveBase = new DriveBase(hardwareMap);
        arm = new Arm(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    // ------------------ Control Logic ------------------ //
    private void handleConfig() {
        if(gamepad1.yWasPressed()) {
            driveBase.resetOtos();
        }
        if(gamepad1.xWasPressed()) {
            driveBase.resetHeading();
            PRM_ENABLED = !PRM_ENABLED;
        }
        if(gamepad1.right_bumper) {
            target_apriltag = 24; // Red
        }
        if(gamepad1.left_bumper) {
            target_apriltag = 20; // Blue
        }
    }

    private void handleDrive() {
        if(PRM_ENABLED) {
            driveBase.globalOmniMoveController(gamepad1);
        }
        else {
            driveBase.omniMoveController(gamepad1);
        }
    }

    private void handleShooter() {
        shooter.SpinnerController(gamepad2, arm_up);
    }

    private void handleArm() {
        arm.ArmController(gamepad2, arm_up);
    }

    private void handleHoming() {
        if(driveBase.aprilTagDetector.last_check.milliseconds() > 10) {
            driveBase.aprilTagDetector.detect();
        }
        if (gamepad1.y) driveBase.moveToPosition(new SparkFunOTOS.Pose2D(0,0,0));
        if(gamepad1.b || gamepad2.b) {
            driveBase.lookAtApriltag(target_apriltag);
        }
    }

    // ------------------ Telemetry ------------------ //
    private void updateTelemetry() {
        telemetry.addData("Target Apriltag", target_apriltag);

        if(target_apriltag == 20) {
            telemetry.addLine("Blue Team");
        }
        else {
            telemetry.addLine("Red Team");
        }
        telemetry.addData("Arm State", arm_up);
        telemetry.addData("Arm Position", arm.Get_Arm_Position());
        telemetry.addData("Apriltag detections", driveBase.aprilTagDetector.current_detections.size());
        telemetry.addData("Ms since last update", driveBase.aprilTagDetector.last_check.milliseconds());
        telemetry.addData("Spinner Current Velocity", shooter.getSpinnerVelocity());
        telemetry.update();
    }
}
