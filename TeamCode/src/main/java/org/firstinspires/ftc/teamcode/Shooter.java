package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private HardwareMap hardwareMap;

    private DcMotorEx leftSpinner, rightSpinner;
    private Servo servo;

    Shooter(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void ServoMoveController(Gamepad gamepad){
        double SPINNER_VELOCITY = 1150;

        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            servo.setPosition(0.8);
        }
        else {
            servo.setPosition(gamepad.a ? 0.8 : 1.0);
        }
    }

    public void SpinnerMoveController(Gamepad gamepad, boolean armUp){
        double SPINNER_VELOCITY = 1150;
        if (gamepad.dpadLeftWasPressed()) SPINNER_VELOCITY -= 10;
        if (gamepad.dpadUpWasPressed()) SPINNER_VELOCITY += 100;
        if (gamepad.dpadRightWasPressed()) SPINNER_VELOCITY += 10;
        if (gamepad.dpadDownWasPressed()) SPINNER_VELOCITY -= 100;
        if (gamepad.right_bumper) {            // Launch
            setSpinnerVelocity(SPINNER_VELOCITY);
        } else if (gamepad.left_bumper) {      // Intake
            if(!armUp) {
                setSpinnerVelocity(-1100);
            }
            else {
                setSpinnerVelocity(-1000);
            }
        } else {                                // Stop
            stopSpinners();
        }

    }

    public double getSpinnerVelocity() {
        return (leftSpinner.getVelocity() + rightSpinner.getVelocity()) / 2.0;
    }

    private void setSpinnerVelocity(double velocity) {
        leftSpinner.setVelocity(velocity);
        rightSpinner.setVelocity(velocity);
    }

    private void stopSpinners() {
        setSpinnerVelocity(0);
    }

}



























