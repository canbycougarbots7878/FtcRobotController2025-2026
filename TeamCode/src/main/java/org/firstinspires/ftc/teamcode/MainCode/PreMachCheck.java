package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Libraries.RobotLib;

@TeleOp(name = "Pre-Mach Check", group = "Check")
public class PreMachCheck extends LinearOpMode {
    private DcMotorEx leftSpinner, rightSpinner;
    private CRServo servo;
    private RobotLib.Robot robot;

    private boolean armUp = false;

    private SparkFunOTOS.Pose2D otos = new SparkFunOTOS.Pose2D(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        waitForStart();

        robot.Set_Arm_Power(1);

        robot.Front_Right.setPower(0.5);
        sleep(1000);
        robot.Front_Right.setPower(0);
        sleep(1000);

        robot.Front_Right.setPower(-0.5);
        sleep(1000);
        robot.Front_Right.setPower(0);
        sleep(1000);

        robot.Front_Left.setPower(0.5);
        sleep(1000);
        robot.Front_Left.setPower(0);
        sleep(1000);

        robot.Front_Left.setPower(-0.5);
        sleep(1000);
        robot.Front_Left.setPower(0);
        sleep(1000);

        robot.Back_Right.setPower(0.5);
        sleep(1000);
        robot.Back_Right.setPower(0);
        sleep(1000);

        robot.Back_Right.setPower(-0.5);
        sleep(1000);
        robot.Back_Right.setPower(0);
        sleep(1000);

        robot.Back_Left.setPower(0.5);
        sleep(1000);
        robot.Back_Left.setPower(0);
        sleep(1000);

        robot.Back_Left.setPower(-0.5);
        sleep(1000);
        robot.Back_Left.setPower(0);
        sleep(1000);

        robot.Arm_Motor.setTargetPosition(640);
        sleep(1000);
        robot.Arm_Motor.setTargetPosition(0);
        sleep(1000);

        leftSpinner.setVelocity(10000);
        sleep(1000);
        leftSpinner.setVelocity(0);
        sleep(1000);

        leftSpinner.setVelocity(-10000);
        sleep(1000);
        leftSpinner.setVelocity(0);
        sleep(1000);

        rightSpinner.setVelocity(10000);
        sleep(1000);
        rightSpinner.setVelocity(0);
        sleep(1000);

        rightSpinner.setVelocity(-10000);
        sleep(1000);
        rightSpinner.setVelocity(0);
        sleep(1000);

        servo.setPower(1);
        sleep(1000);
        servo.setPower(0);
        sleep(1000);

        servo.setPower(-1);
        sleep(1000);
        servo.setPower(0);
        sleep(1000);


        telemetry.addData("X", otos.x);
        telemetry.addData("Y", otos.y);
        telemetry.addData("H", otos.h);
        telemetry.update();
        sleep(1000);

    }

    private void initHardware() {
        robot = new RobotLib.Robot(hardwareMap)
                .enableArm()
                .enableAprilTagDetection()
                .enableOtos()
                .enableIMU();

        robot.Reverse_Left();
        //robot.Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(CRServo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
    }


}
