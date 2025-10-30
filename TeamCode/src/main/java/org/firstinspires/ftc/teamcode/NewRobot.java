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

    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap).enableArm().enableAprilTagDetection().enableOtos(); // Initialize Robot handler without Otos and with arm enabled
        robot.Reverse_Left(); // Make all motors spin forward

        servo = hardwareMap.get(Servo.class, "servo");

        lSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        double spinnerspeed = 1400;
        //lSpinner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //rSpinner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        double speed = 0.5;

        boolean armUp = false;

        waitForStart();
        robot.Set_Arm_Power(1);
        while(opModeIsActive()) {
            robot.Omni_Move_Controller(gamepad1,speed);
            if(gamepad1.a) {
                servo.setPosition(0.85);
            }
            else {
                servo.setPosition(1);
            }

            // Arm Control
            if(gamepad1.xWasPressed()) {
                armUp = !armUp;
                if(armUp) {
                    robot.Arm_Motor.setTargetPosition(640);
                }
                else {
                    robot.Arm_Motor.setTargetPosition(0);
                }
            }
            int armPos = robot.Arm_Motor.getCurrentPosition();
            if(!gamepad1.x && armPos < 30) {
                robot.Set_Arm_Power(0);
                if(armPos < 0) {
                    robot.Reset_Arm_Reading();
                }
            }
            else {
                robot.Set_Arm_Power(1);
            }

            // Homing code
            if(gamepad1.start) robot.Reset_Otos();
            if(gamepad1.y) robot.Return_Home();

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
