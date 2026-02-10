package org.firstinspires.ftc.teamcode.Libraries;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveHubTelemetry {
    Telemetry telemetry;

    public void DriveHubTelemetrySetUp(){
        telemetry.addLine("Everything is set up");
        telemetry.update();
    }


}
