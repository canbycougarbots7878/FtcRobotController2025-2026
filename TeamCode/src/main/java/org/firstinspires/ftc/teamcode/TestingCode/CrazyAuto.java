package org.firstinspires.ftc.teamcode.TestingCode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.Arm;
import org.firstinspires.ftc.teamcode.Libraries.DriveBase;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

@Autonomous(name = "I'm losing my mind", group = "Test")
public class CrazyAuto extends LinearOpMode {
    DriveBase driveBase = null; // Declare drive base
    SparkFunOTOS.Pose2D shooting_pos = new SparkFunOTOS.Pose2D(15,-15,29);
    boolean in_position = false;

    public void runOpMode() {
        driveBase = new DriveBase(hardwareMap);
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
            }

            //
            if(driveBase.aprilTagDetector.last_check.milliseconds() > 10 && in_position) {
                driveBase.aprilTagDetector.detect();
            }
            if (in_position) {
                arm.Set_Arm_Power(1);
                arm.ArmAuto(true);
                if (arm.isArmUp()) {
                    shooter.SpinnerAuto(arm.isArmUp(), true, false);
                }
            }
            driveBase.telemetryPosition(telemetry);
            telemetry.addData("Distance",driveBase.distanceTo(shooting_pos,false));
            telemetry.addData("In position",in_position);
            telemetry.update();
        }

    }
}
