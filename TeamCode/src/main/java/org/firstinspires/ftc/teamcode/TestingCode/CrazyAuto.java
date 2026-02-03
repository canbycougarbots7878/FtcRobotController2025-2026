package org.firstinspires.ftc.teamcode.TestingCode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Libraries.Arm;
import org.firstinspires.ftc.teamcode.Libraries.DriveBase;
import org.firstinspires.ftc.teamcode.Libraries.RobotLib;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "I'm losing my mind", group = "Test")
public class CrazyAuto extends LinearOpMode {
    DriveBase driveBase = null; // Declare drive base
    SparkFunOTOS.Pose2D shooting_pos = new SparkFunOTOS.Pose2D(15,-15,29);
    private RobotLib.Robot robot;
    boolean in_position = false;
    boolean aimed = false;
    int target_apriltag = 20;

    public void runOpMode() {
        driveBase = new DriveBase(hardwareMap);
        robot = new RobotLib.Robot(hardwareMap).enableAprilTagDetection();

        Arm arm = new Arm(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            // Get in position for shooting
            if(driveBase.distanceTo(shooting_pos, true) > 1 && !in_position) {
                driveBase.moveToPosition(shooting_pos);
            }
            else {
                in_position = true;
                driveBase.stop();
            }

            //
            if(driveBase.aprilTagDetector.last_check.milliseconds() > 10 && in_position) {
                driveBase.aprilTagDetector.detect();
            }


            if (in_position && aimed) {
                arm.Set_Arm_Power(1);
                arm.ArmAuto(true);
                if (arm.isArmUp()) {
                    shooter.SpinnerAuto(arm.isArmUp(), true, false);
                }
            }
            driveBase.telemetryPosition(telemetry);
            telemetry.addData("Distance",driveBase.distanceTo(shooting_pos,false));
            telemetry.addData("In position",in_position);
            telemetry.addData("spinner velocity", shooter.getSpinnerVelocity());
            telemetry.addData("Is arm up", arm.isArmUp());
            telemetry.update();
        }

    }

    private void handleHoming(){
        if (robot.timeSinceLastAprilTagCheck.milliseconds() > 10) {
            robot.UpdateAprilTagDetections();
        }
        robot.LookAtAprilTag(target_apriltag,10);
    }
}
