package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libraries.Arm;
import org.firstinspires.ftc.teamcode.Libraries.DriveBase;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;

@Autonomous(name = "Autonomous Blue", group = "Robot")
public class CrazyAuto extends LinearOpMode {
    DriveBase driveBase = null; // Declare drive base
    SparkFunOTOS.Pose2D shooting_pos = new SparkFunOTOS.Pose2D(15,-15,29);
    SparkFunOTOS.Pose2D moving_pos = new SparkFunOTOS.Pose2D(7.6,-59.4,0);
    boolean in_position = false;
    boolean aimed = false;
    int target_apriltag = 20;

    ElapsedTime waittimer;
    boolean waiting = false;

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
                if(!waiting) {
                    driveBase.stop();
                }
            }

            // Scan for new april tags every 10 milliseconds
            if(driveBase.aprilTagDetector.last_check.milliseconds() > 10 && in_position) {
                driveBase.aprilTagDetector.detect();
            }

            // If robot is in position to shoot, then shoot
            if (in_position) {
                driveBase.lookAtApriltag(target_apriltag,10);
                arm.Set_Arm_Power(1);
                arm.ArmAuto(true);
                if (arm.isArmUp()) {
                    shooter.SpinnerAuto(arm.isArmUp(), true, false,1075);
                    if(!waiting) {
                        waittimer = new ElapsedTime();
                        driveBase.stop();
                    }
                    waiting = true;
                    driveBase.stop();
                }
                if(waiting && waittimer.seconds() > 5) {
                    arm.Set_Arm_Power(0);
                    shooter.stopSpinners();
                    if(driveBase.distanceTo(moving_pos, true) > 1) {
                        driveBase.moveToPosition(moving_pos);
                    }
                    else {
                        driveBase.stop();
                        requestOpModeStop();
                    }
                }
            }

            // Telemetry
            driveBase.telemetryPosition(telemetry);
            telemetry.addData("Distance",driveBase.distanceTo(shooting_pos,false));
            telemetry.addData("In position",in_position);
            telemetry.addData("spinner velocity", shooter.getSpinnerVelocity());
            telemetry.addData("Is arm up", arm.isArmUp());
            telemetry.update();
        }

    }

}
