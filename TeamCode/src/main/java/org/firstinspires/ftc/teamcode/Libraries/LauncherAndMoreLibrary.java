package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherAndMoreLibrary {
    ArmLibrary armLibrary;
    ShooterLibrary shooterLibrary;

    public void LauncherAndMoreSetUp(HardwareMap hardwareMap){
        armLibrary.ArmSetUp(hardwareMap);
        shooterLibrary.ShooterSetUp(hardwareMap);
    }

}
