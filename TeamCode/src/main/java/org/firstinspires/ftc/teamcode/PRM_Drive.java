package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Player Relative Drive", group = "Movement")
public class PRM_Drive extends LinearOpMode {
    MovementLib.Robot robot = null;
    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap).enableIMU(); // Initialize Wheels handler
        robot.Reverse_Right(); // Make all motors spin forward

        waitForStart();
        while(opModeIsActive()) {
            robot.PRM_Move(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, 0.75);

            if(gamepad1.start) robot.imu.resetYaw();
        }
    }
}
