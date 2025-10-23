package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto", group="Robot")
public class quickAuto extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    MovementLib.Robot robot = null;

    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        robot = new MovementLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left);

        robot.Reverse_Left();

        waitForStart();

        if (opModeIsActive()) {
            robot.Omni_Move( 0.5, 0, 0, 1.0);

            sleep(1500);

            robot.Omni_Move( 0, 0, 0, 0.0);

        }
    }

}
