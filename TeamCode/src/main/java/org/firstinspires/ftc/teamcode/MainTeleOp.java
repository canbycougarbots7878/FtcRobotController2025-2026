package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.Reader;
import java.util.Scanner;

@TeleOp(name = "Main TeleOp", group = "Official")
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx leftSpinner, rightSpinner;
    private Servo servo;
    private RobotLib.Robot robot;

    private boolean PRM_ENABLED = false;
    private boolean armUp = false;
    private double driveSpeed = 0.5;
    private double SPINNER_VELOCITY = 1150;
    private boolean debug_mode = false;
    private int target_apriltag = 20;

    @Override
    public void runOpMode() {
        initHardware();
        //ReadSpeedConfig();
        waitForStart();

        robot.Set_Arm_Power(1);

        while (opModeIsActive()) {
            if(gamepad1.back) {
                handleConfig();
            }
            else {
                handleServo();
                handleArm();
                handleHoming();
            }
            handleSpinners();
            handleDrive();
            updateTelemetry();
        }
    }

    // ------------------ Initialization ------------------ //
    private void initHardware() {
        robot = new RobotLib.Robot(hardwareMap)
                .enableArm()
                .enableAprilTagDetection()
                .enableOtos()
                .enableIMU();

        robot.Reverse_Left();
        //robot.Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ------------------ Control Logic ------------------ //
    private void handleConfig() {
        if(gamepad1.yWasPressed()) {
            robot.Reset_Otos();
        }
        if(gamepad1.xWasPressed()) {
            robot.Reset_IMU();
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
            robot.PRM_Move_Controller(gamepad1, 0.5f + gamepad1.left_trigger / 2.0f);
        }
        else {
            robot.Omni_Move_Controller(gamepad1, 0.5f + gamepad1.left_trigger / 2.0f);
        }
    }

    private void handleServo() {
        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            servo.setPosition(0.8);
        }
        else {
            servo.setPosition(gamepad1.a ? 0.8 : 1.0);
        }
    }

    private void handleArm() {
        // Toggle arm position when X is pressed
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            armUp = !armUp;
            robot.Arm_Motor.setTargetPosition(armUp ? (gamepad1.left_bumper || gamepad2.left_bumper ? 600 : 640) : 0);
        }

        int armPos = robot.Arm_Motor.getCurrentPosition();

        // Arm stopping and reset behavior
        if (!(gamepad1.x || gamepad2.x) && armPos < 30) {
            robot.Set_Arm_Power(0);
            if (armPos < 10) {
                robot.Reset_Arm_Reading();
            }
        } else {
            robot.Set_Arm_Power(1);
        }

        if (gamepad1.back) {
            robot.Reset_Arm_Reading();
        }
    }

    private void handleHoming() {
        if (robot.timeSinceLastAprilTagCheck.milliseconds() > 10) {
            robot.UpdateAprilTagDetections();
        }
        if (gamepad1.y) robot.Return_Home();
        if(gamepad1.b || gamepad2.b) {
            robot.LookAtAprilTag(20);
        }
    }

    private void handleSpinners() {
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) SPINNER_VELOCITY -= 10;
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) SPINNER_VELOCITY += 100;
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) SPINNER_VELOCITY += 10;
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) SPINNER_VELOCITY -= 100;
        if (gamepad1.right_bumper || gamepad2.right_bumper) {            // Launch
            driveSpeed = 0.4;
            setSpinnerVelocity(SPINNER_VELOCITY);
        } else if (gamepad1.left_bumper || gamepad2.left_bumper) {      // Intake
            driveSpeed = 0.5;
            if(!armUp) {
                setSpinnerVelocity(-1100);
            }
            else {
                setSpinnerVelocity(-1000);
            }
        } else {                                // Stop
            driveSpeed = 0.5;
            stopSpinners();
        }
    }

    // ------------------ Spinner Helpers ------------------ //
    private void setSpinnerVelocity(double velocity) {
        leftSpinner.setVelocity(velocity);
        rightSpinner.setVelocity(velocity);
    }

    private double getSpinnerVelocity() {
        return (leftSpinner.getVelocity() + rightSpinner.getVelocity()) / 2.0;
    }

    private void setSpinnerPower(double power) {
        leftSpinner.setPower(power);
        rightSpinner.setPower(power);
    }

    private void stopSpinners() {
        setSpinnerPower(0);
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
        telemetry.addData("Arm State", armUp);
        telemetry.addData("Arm Position", robot.Arm_Motor.getCurrentPosition());
        telemetry.addData("Arm Target", robot.Arm_Motor.getTargetPosition());
        telemetry.addData("Apriltag detections", robot.currentDetections.size());
        telemetry.addData("Ms since last update", robot.timeSinceLastAprilTagCheck.milliseconds());
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Spinner Current Velocity", getSpinnerVelocity());
        telemetry.addData("Spinner Target Velocity", SPINNER_VELOCITY);
        telemetry.update();
    }

    // Config file
    private void ReadSpeedConfig() {
        File file = new File("speed.txt");
        try (Scanner reader = new Scanner(file)) {
            String data = reader.nextLine();
            SPINNER_VELOCITY = Integer.parseInt(data);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
