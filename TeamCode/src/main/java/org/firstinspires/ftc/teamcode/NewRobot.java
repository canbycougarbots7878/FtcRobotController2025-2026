package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "New Robot Test", group = "Official")
public class NewRobot extends LinearOpMode {
    DcMotor lSpinner = null;
    DcMotor rSpinner = null;
    MovementLib.Robot robot = null;

    Servo servo = null;

    int tick = 0;
    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap).enableArm().enableAprilTagDetection(); // Initialize Robot handler without Otos and with arm enabled
        robot.Reverse_Left(); // Make all motors spin forward

        servo = hardwareMap.get(Servo.class, "servo");

        lSpinner = hardwareMap.get(DcMotor.class, "leftspinner");
        rSpinner = hardwareMap.get(DcMotor.class, "rightspinner");
        rSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        robot.Set_Arm_Power(1);
        while(opModeIsActive()) {
            double speed = (gamepad1.right_bumper ? 1.0 : 0.5);
            robot.Omni_Move_Controller(gamepad1,speed);
            if(gamepad1.a) {
                servo.setPosition(0.25);
            }
            else {
                servo.setPosition(0);
            }

            double arm_movement = gamepad1.right_trigger - gamepad1.left_trigger; // ranges from 0 to 1
            tick += (int)(arm_movement * 4);
            if(tick < 30 && (gamepad1.right_trigger + gamepad1.left_trigger) == 0) {
                telemetry.addLine("Arm power save mode");
                robot.Set_Arm_Power(0); // Power save
                if(tick < 0) {
                    robot.Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Arm_Motor.setTargetPosition(0);
                    robot.Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else {
                robot.Set_Arm_Power(1);
            }

            robot.Set_Arm_Position(tick);

            if(gamepad1.right_bumper) { // Launch
                lSpinner.setPower(0.8);
                rSpinner.setPower(0.8);
            }
            else if(gamepad1.left_bumper) { // Intake
                lSpinner.setPower(-.75);
                rSpinner.setPower(-.75);
            }
            else {
                lSpinner.setPower(0);
                rSpinner.setPower(0);
            }

            telemetry.addData("Target Position", robot.Arm_Motor.getTargetPosition());
            telemetry.addData("Actual Position", robot.Get_Arm_Position());
            telemetry.update();
        }
    }
}
