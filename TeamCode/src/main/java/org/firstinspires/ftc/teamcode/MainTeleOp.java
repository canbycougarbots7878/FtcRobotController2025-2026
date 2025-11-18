package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp", group = "Official")
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx leftSpinner, rightSpinner;
    private Servo servo;
    private MovementLib.Robot robot;

    private boolean armUp = false;
    private double driveSpeed = 0.5;
    private double SPINNER_VELOCITY = 1010;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        robot.Set_Arm_Power(1);

        while (opModeIsActive()) {
            handleDrive();
            handleServo();
            handleArm();
            handleHoming();
            handleSpinners();
            updateTelemetry();
        }
    }

    // ------------------ Initialization ------------------ //
    private void initHardware() {
        robot = new MovementLib.Robot(hardwareMap)
                .enableArm()
                .enableAprilTagDetection()
                .enableOtos();

        robot.Reverse_Left();
        robot.Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ------------------ Control Logic ------------------ //

    private void handleDrive() {
        robot.Omni_Move_Controller(gamepad1, 0.5f + gamepad1.left_trigger / 2.0f);
    }

    private void handleServo() {
        // Servo control (A = close, default = open)
        servo.setPosition(gamepad1.a || leftSpinner.getVelocity() > SPINNER_VELOCITY ? 0.7 : 1.0);
    }

    private void handleArm() {
        // Toggle arm position when X is pressed
        if (gamepad1.xWasPressed()) {
            armUp = !armUp;
            robot.Arm_Motor.setTargetPosition(armUp ? 640 : 0);
        }

        int armPos = robot.Arm_Motor.getCurrentPosition();

        // Arm stopping and reset behavior
        if (!gamepad1.x && armPos < 30) {
            robot.Set_Arm_Power(0);
            if (armPos < 0) {
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
        if (gamepad1.start) robot.Reset_Otos();
        if (gamepad1.y) robot.Return_Home();
    }

    private void handleSpinners() {
        if (gamepad1.dpadLeftWasPressed()) SPINNER_VELOCITY -= 10;
        if (gamepad1.dpadUpWasPressed()) SPINNER_VELOCITY += 100;
        if (gamepad1.dpadRightWasPressed()) SPINNER_VELOCITY += 10;
        if (gamepad1.dpadDownWasPressed()) SPINNER_VELOCITY -= 100;
        if (gamepad1.right_bumper) {            // Launch
            driveSpeed = 0.4;
            setSpinnerVelocity(SPINNER_VELOCITY);
        } else if (gamepad1.left_bumper) {      // Intake
            driveSpeed = 0.5;
            setSpinnerPower(-0.45);
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

    private void setSpinnerPower(double power) {
        leftSpinner.setPower(power);
        rightSpinner.setPower(power);
    }

    private void stopSpinners() {
        setSpinnerPower(0);
    }

    // ------------------ Telemetry ------------------ //
    private void updateTelemetry() {
        telemetry.addData("Arm Target", robot.Arm_Motor.getTargetPosition());
        telemetry.addData("Arm Actual", robot.Get_Arm_Position());
        telemetry.addData("Arm Up", armUp);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Launch Speed", SPINNER_VELOCITY);
        telemetry.update();
    }
}
