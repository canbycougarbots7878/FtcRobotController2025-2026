package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Basic Drive", group = "Movement")
public class Basic_Drive extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;
    MovementLib.Robot robot = null;
    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap); // Initialize Wheels handler
        robot.Reverse_Right(); // Make all motors spin forward

        waitForStart();
        while(opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = - gamepad1.left_stick_x;
            double speed = (gamepad1.right_bumper ? 1.0 : 0.5);
            robot.Omni_Move(forward, strafe, gamepad1.right_stick_x, speed);
        }
    }
}
