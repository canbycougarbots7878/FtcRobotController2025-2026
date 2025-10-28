package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "New Robot Test", group = "Official")
public class NewRobot extends LinearOpMode {
    DcMotorEx lSpinner = null;
    DcMotorEx rSpinner = null;
    MovementLib.Robot robot = null;

    Servo servo = null;

    int tick = 0;
    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap).enableArm().enableAprilTagDetection(); // Initialize Robot handler without Otos and with arm enabled
        robot.Reverse_Left(); // Make all motors spin forward

        servo = hardwareMap.get(Servo.class, "servo");

        lSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        double spinnerspeed = 1600;
        //lSpinner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //rSpinner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        double speed = 0.5;

        waitForStart();
        robot.Set_Arm_Power(1);
        while(opModeIsActive()) {
            robot.Omni_Move_Controller(gamepad1,speed);
            if(gamepad1.a) {
                servo.setPosition(0);
            }
            else {
                servo.setPosition(0.25);
            }

            double arm_movement = gamepad1.right_trigger - gamepad1.left_trigger; // ranges from 0 to 1
            tick += (int)(arm_movement * 4);
            if(tick < 30 && (gamepad1.right_trigger + gamepad1.left_trigger) == 0) {
                telemetry.addLine("Arm power save mode");
                robot.Set_Arm_Power(0); // Power save
                if(tick < 0) {
                    robot.Arm_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    robot.Arm_Motor.setTargetPosition(0);
                    robot.Arm_Motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
            }
            else {
                robot.Set_Arm_Power(1);
            }

            robot.Set_Arm_Position(tick);

            if(gamepad1.right_bumper) { // Launch
                speed = 0.4;
                if(gamepad1.y) {
                    telemetry.addLine("Spinner Overdrive");
                    lSpinner.setVelocity(2000);
                    rSpinner.setVelocity(2000);
                }
                else {
                    lSpinner.setVelocity(spinnerspeed);
                    rSpinner.setVelocity(spinnerspeed);
                }
            }
            else if(gamepad1.left_bumper) { // Intake
                speed = 0.5;
                lSpinner.setPower(-.75);
                rSpinner.setPower(-.75);
            }
            else {
                speed = 0.5;
                lSpinner.setPower(0);
                rSpinner.setPower(0);
            }
            telemetry.addData("Target Position", robot.Arm_Motor.getTargetPosition());
            telemetry.addData("Actual Position", robot.Get_Arm_Position());
            telemetry.update();
        }
    }
}
