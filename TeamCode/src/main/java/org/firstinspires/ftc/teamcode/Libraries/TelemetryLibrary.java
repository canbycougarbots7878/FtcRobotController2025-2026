package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TelemetryLibrary {
    ColorLibrary colorLibrary;

    public void TelemetrySetUp(HardwareMap hardwareMap){
        colorLibrary.ColorServoSetUp(hardwareMap);
    }
}
