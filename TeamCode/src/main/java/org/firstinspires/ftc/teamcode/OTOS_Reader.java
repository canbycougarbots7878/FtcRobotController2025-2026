package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "OTOS Reader", group = "Sensor")
public class OTOS_Reader extends LinearOpMode {
    SparkFunOTOS myOtos;
    MovementLib.Robot robot;


    public SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0);
    public void runOpMode() {
        robot = new MovementLib.Robot(hardwareMap, true, false); // Init Robot with Otos enabled
        robot.Reverse_Left(); // Make all motors spin forward


        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();


        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        waitForStart();
        while(opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            if(gamepad1.start) myOtos.resetTracking();


            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);


            robot.Omni_Move_To_Target(target);


            // Update the telemetry on the driver station
            telemetry.update();
        }
    }
}
