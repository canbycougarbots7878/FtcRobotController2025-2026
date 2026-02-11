package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmLibrary {
    HardwareMap hardwareMap;

    DcMotor Arm_Motor;

    RobotLibrary robotLibrary;

    public void ArmSetUp(HardwareMap hardwareMap){
        // Arm initialization
        this.Arm_Motor = hardwareMap.get(DcMotor.class, "arm");
        this.Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Arm_Motor.setTargetPosition(0);
        this.Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     *
     * @param gamepad gamepad num
     * @param armUp  add "if (gamepad.xWasPressed() || gamepad.xWasPressed()) {
     *             armUp = !armUp;
     *               }" before this line of code (and set it at the start to false)
     */

    public void ArmController(Gamepad gamepad, boolean armUp) {
        // Toggle arm position when X is pressed
        if (gamepad.xWasPressed() || gamepad.xWasPressed()) {
            this.Arm_Motor.setTargetPosition(armUp ? (gamepad.left_bumper ? 600 : 640) : 0);

            if (armUp) {
                double green = robotLibrary.telemetryLibrary.colorLibrary.GREEN;
                robotLibrary.telemetryLibrary.colorLibrary.setColor(green);
            } else {
                double red = robotLibrary.telemetryLibrary.colorLibrary.RED;
                robotLibrary.telemetryLibrary.colorLibrary.setColor(red);
            }
        }

        int armPos = this.Arm_Motor.getCurrentPosition();

        // Arm stopping and reset behavior
        if (!(gamepad.x) && armPos < 30) {
            this.Set_Arm_Power(0);
            if (armPos < 10) {
                this.Reset_Arm_Reading();
            }
        } else {
            this.Set_Arm_Power(1);
        }

        if (gamepad.back) {
            this.Reset_Arm_Reading();
        }

    }


    public boolean isArmUp(){
        if (Math.abs(Get_Arm_Position() - 600) <= 10){
            return true;
        }else {
            return false;
        }
    }

    public void Set_Arm_Power(double power) {
        this.Arm_Motor.setPower(power);
    }

    public void Set_Arm_Position(int tick) {
        this.Arm_Motor.setTargetPosition(tick);
    }

    public int Get_Arm_Position() {
        return this.Arm_Motor.getCurrentPosition();
    }

    public void Reset_Arm_Reading() {
        Arm_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setTargetPosition(0);
        Arm_Motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void ArmAuto(boolean armUp){
        this.Arm_Motor.setTargetPosition(armUp ? 600 : 0);

        if (armUp) {
            double green = robotLibrary.telemetryLibrary.colorLibrary.GREEN;
            robotLibrary.telemetryLibrary.colorLibrary.setColor(green);
        } else {
            double red = robotLibrary.telemetryLibrary.colorLibrary.RED;
            robotLibrary.telemetryLibrary.colorLibrary.setColor(red);
        }
    }


}
