package org.firstinspires.ftc.teamcode.TestingCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libraries.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Libraries.Arm;
import org.firstinspires.ftc.teamcode.Libraries.DriveBase;
import org.firstinspires.ftc.teamcode.Libraries.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Autonomous Testing", group = "Test")
public class AutoTesting extends LinearOpMode {
    DriveBase driveBase = null;
    Arm arm = null;
    Shooter shooter = null;
    int target_apriltag = 20; // Blue team
    Boolean arm_up = false;


    @Override
    public void runOpMode(){
        initHardware();

        int rotateCounter = 0;
        int rotateCounterLimit = 100;

        boolean targetPoseAchieved = false;
        double RobotTurn = 0.1;

        AprilTagDetector aprilTagDetector = null;
        aprilTagDetector = new AprilTagDetector(hardwareMap);

        Boolean aprilTagDetected = aprilTagDetector.findByIDBol(20) || aprilTagDetector.findByIDBol(24);

        if (aprilTagDetected){



        }

        waitForStart();

        while (opModeIsActive()){

        }




    }

    private void initHardware() {
        driveBase = new DriveBase(hardwareMap);
        arm = new Arm(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    private void handleDrive() {


    }

}







































