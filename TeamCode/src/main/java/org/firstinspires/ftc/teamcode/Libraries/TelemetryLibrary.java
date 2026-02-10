package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TelemetryLibrary {
    ColorLibrary colorLibrary;
    DriveHubTelemetry driveHubTelemetry;

    public void TelemetrySetUp(HardwareMap hardwareMap){
        colorLibrary.ColorServoSetUp(hardwareMap);
        driveHubTelemetry.DriveHubTelemetrySetUp();
    }
}
