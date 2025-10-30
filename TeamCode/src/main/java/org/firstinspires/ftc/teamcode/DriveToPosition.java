package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive to position", group="Testing")
public class DriveToPosition extends LinearOpMode {
    // Runs once (Load)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    double accuracy = 40;
    public void runOpMode() {
        MovementLib.Robot robot = new MovementLib.Robot(hardwareMap).enableAprilTagDetection().enableIMU().enableOtos();
        robot.Reverse_Left();
        waitForStart();
        while (opModeIsActive()) {
            if(robot.timeSinceLastAprilTagCheck.time() > 0.1) {
                robot.UpdateAprilTagDetections();
            }
            if(gamepad1.a) {
                SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(0,0,0);
                robot.Omni_Move_To_Target(pose,0.5);
            }
            else {
                double speed = (gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.25 : 0.5));
                robot.Omni_Move_Controller(gamepad1, speed);
            }

            if(gamepad1.start) {
                robot.Reset_Otos();
            }
            if(robot.timeSinceLastAprilTagCheck.milliseconds() < 100) {
                robot.UpdateAprilTagDetections();
            }
            //robot.TelemetryAprilTags(telemetry);
            SparkFunOTOS.Pose2D pos = robot.GetPositionBasedOnAprilTag();
            MovementLib.Pose2DSetHeading(pos, 0);
            if(pos.x + pos.y + pos.h != 0) {
                telemetry.addData("X", pos.x);
                telemetry.addData("Y", pos.y);
                telemetry.addData("H", pos.h);
            }
            telemetry.update();
        }
    }
}
