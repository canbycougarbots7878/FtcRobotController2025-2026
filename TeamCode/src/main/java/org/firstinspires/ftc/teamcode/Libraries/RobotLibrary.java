package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotLibrary {
    MovementLibrary movementLibrary;

    public void RobotSetUp(HardwareMap hardwareMap){
        movementLibrary.MovementLibraryCreation(hardwareMap);
    }

}
