package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelControl {
    private HardwareMap hardwareMap;

    public DcMotor front_right;
    public DcMotor front_left;
    public DcMotor back_right;
    public DcMotor back_left;

    // Constructor
    public void DriveBase(HardwareMap hardwareMap)  {
        this.front_right = hardwareMap.get(DcMotor.class, "frontright");
        this.front_left = hardwareMap.get(DcMotor.class, "frontleft");
        this.back_right = hardwareMap.get(DcMotor.class, "backright");
        this.back_left = hardwareMap.get(DcMotor.class, "backleft");

        // Reverse left motors so all motors spin same direction
        this.front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        this.back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hardwareMap = hardwareMap;
    }

    // Raw wheel control
    public void setWheelPowers(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
        this.front_right.setPower(Front_Right_Power);
        this.front_left.setPower(Front_Left_Power);
        this.back_right.setPower(Back_Right_Power);
        this.back_left.setPower(Back_Left_Power);
    }
}
