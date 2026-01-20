package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Shooter {
    private HardwareMap hardwareMap;

    private DcMotorEx leftSpinner, rightSpinner;
    private CRServo servo;

    Shooter(HardwareMap hardwareMap){
        servo = hardwareMap.get(CRServo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void LaunchServoController(Gamepad gamepad){
        double SPINNER_VELOCITY = 1150;

        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            servo.setPower(1);
        }
        else {
            servo.setPower(gamepad.a ? 1 : 0);
        }
    }

    public void inputServoController(Gamepad gamepad, double SPINNER_VELOCITY){


        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() < SPINNER_VELOCITY-20) {
            servo.setPower(-1);
        }
        else {
            servo.setPower(gamepad.a ? -1 : 0);
        }
    }

    public void inputServoAuto(double SPINNER_VELOCITY){


        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() < SPINNER_VELOCITY-20) {
            servo.setPower(-1);
        }
        else {
            servo.setPower(0);
        }
    }

    public void LaunchServoAuto(){
        double SPINNER_VELOCITY = 1150;

        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            servo.setPower(1);
        }
        else {
            servo.setPower(0);
        }
    }

    public void SpinnerController(Gamepad gamepad, boolean armUp){
        double SPINNER_VELOCITY = 1150;
        if (gamepad.dpadLeftWasPressed()) SPINNER_VELOCITY -= 10;
        if (gamepad.dpadUpWasPressed()) SPINNER_VELOCITY += 100;
        if (gamepad.dpadRightWasPressed()) SPINNER_VELOCITY += 10;
        if (gamepad.dpadDownWasPressed()) SPINNER_VELOCITY -= 100;
        if (gamepad.right_bumper) {            // Launch
            setSpinnerVelocity(SPINNER_VELOCITY);
            LaunchServoController(gamepad);
        } else if (gamepad.left_bumper) {      // Intake
            if(!armUp) {
                setSpinnerVelocity(-1100);
                inputServoController(gamepad,-1100);
            }
            else {
                setSpinnerVelocity(-1000);
                inputServoController(gamepad,-1000);
            }
        } else {                                // Stop
            stopSpinners();
        }

    }

    /**
     *
     * @param armUp for inputs ( up makes the velocity slower than down).
     * @param launch if true and input is false launch the ball.
     * @param input if true and launch is false intake the ball.
     *              if both are the same spinners stop.
     */
    public void SpinnerAuto(boolean armUp, boolean launch, boolean input){
        double SPINNER_VELOCITY = 1150;
        if (launch && !input) {            // Launch
            setSpinnerVelocity(SPINNER_VELOCITY);
            LaunchServoAuto();
        } else if (!launch && input) {      // Intake
            if(!armUp) {
                setSpinnerVelocity(-1100);
                inputServoAuto(-1100);
            }
            else {
                setSpinnerVelocity(-1000);
                inputServoAuto(-1000);
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



























