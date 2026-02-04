package org.firstinspires.ftc.teamcode.TestingCode;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.DriveBase;

@Disabled
@TeleOp(name = "Limit_Testing", group = "Official")
public class Limit_Testing extends LinearOpMode {
    DriveBase driveBase = null;
    double target_heading = 0;
    int stage = 0;
    SparkFunOTOS.Pose2D[] pose = {new SparkFunOTOS.Pose2D(10,0,0),new SparkFunOTOS.Pose2D(10,10,90),new SparkFunOTOS.Pose2D(0,10,0),new SparkFunOTOS.Pose2D(0,0,90)};
    public void runOpMode() {
        driveBase = new DriveBase(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            driveBase.moveToPosition(pose[stage]);
            if(driveBase.distanceTo(pose[stage],true) < 1) {
                stage = (stage + 1) % 4;
            }
            driveBase.telemetryPosition(telemetry);
            telemetry.update();
        }
    }
}