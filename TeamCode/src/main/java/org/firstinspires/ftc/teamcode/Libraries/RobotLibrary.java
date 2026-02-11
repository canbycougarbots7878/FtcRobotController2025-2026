package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotLibrary {
    MovementLibrary movementLibrary;
    TelemetryLibrary telemetryLibrary;
    LauncherAndMoreLibrary launcherAndMoreLibrary;

    public void RobotSetUp(HardwareMap hardwareMap){
        movementLibrary.MovementLibraryCreation(hardwareMap);
        telemetryLibrary.TelemetrySetUp(hardwareMap);
        launcherAndMoreLibrary.LauncherAndMoreSetUp(hardwareMap);
    }

}
