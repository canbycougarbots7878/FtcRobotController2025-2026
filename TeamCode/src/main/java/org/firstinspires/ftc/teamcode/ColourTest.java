package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Colour test", group = "Test")
public class ColourTest extends LinearOpMode {
    ServoLED LED = null;

    @Override
    public void runOpMode() {
        LED = new ServoLED(hardwareMap, "LED");

        waitForStart();
        LED.setColor(LED.GREEN);
        sleep(3000);

        while (opModeIsActive()){
            LED.setColor(0.3);

        }
    }

}




















