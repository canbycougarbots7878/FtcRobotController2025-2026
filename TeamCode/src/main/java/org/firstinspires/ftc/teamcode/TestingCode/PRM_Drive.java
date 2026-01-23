package org.firstinspires.ftc.teamcode.TestingCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.RobotLib;

@Disabled
@TeleOp(name = "Player Relative Drive", group = "Movement")
public class PRM_Drive extends LinearOpMode {
    RobotLib.Robot robot = null;
    public void runOpMode() {
        robot = new RobotLib.Robot(hardwareMap).enableIMU(); // Initialize Wheels handler
        robot.Reverse_Left(); // Make all motors spin forward

        waitForStart();
        while(opModeIsActive()) {
            double speed = (gamepad1.right_bumper ? 1.0 : 0.5);
            robot.PRM_Move_Controller(gamepad1, speed);
            if(gamepad1.start) robot.imu.resetYaw();
        }
    }
}
