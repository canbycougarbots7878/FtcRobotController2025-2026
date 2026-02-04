package org.firstinspires.ftc.teamcode.TestingCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.ServoLED;

@Disabled
@TeleOp(name = "Colour test", group = "Test")
public class ColourTest extends LinearOpMode {
    ServoLED LED = null;

    @Override
    public void runOpMode() {
        LED = new ServoLED(hardwareMap, "LED");
        double colorNum = 0.2;

        waitForStart();

        while (opModeIsActive()){


            if (colorNum <= 0.75){
                LED.setColor(colorNum);

                telemetry.addData("colour value", colorNum);
                telemetry.update();

                colorNum += 0.0001;

                sleep(10);

            }
        }
    }

}




















