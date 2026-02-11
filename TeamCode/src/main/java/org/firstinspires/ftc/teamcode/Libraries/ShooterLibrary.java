package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterLibrary {
    HardwareMap hardwareMap;

    DcMotorEx leftSpinner, rightSpinner;
    CRServo pusher;

    public void ShooterSetUp(HardwareMap hardwareMap){
        pusher = hardwareMap.get(CRServo.class, "servo");

        leftSpinner = hardwareMap.get(DcMotorEx.class, "leftspinner");
        rightSpinner = hardwareMap.get(DcMotorEx.class, "rightspinner");
        rightSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void LaunchPusherController(Gamepad gamepad){
        double SPINNER_VELOCITY = 1150;

        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            pusher.setPower(1);
            //led.setColor(led.RED);
        }
        else {
            pusher.setPower(gamepad.a ? 1 : 0);
        }
    }

    public void inputPusherController(Gamepad gamepad, double SPINNER_VELOCITY){
        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() < SPINNER_VELOCITY-20) {
            pusher.setPower(-1);
        }
        else {
            pusher.setPower(gamepad.a ? -1 : 0);
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
            LaunchPusherController(gamepad);
        } else if (gamepad.left_bumper) {      // Intake
            if(!armUp) {
                setSpinnerVelocity(-1100);
                inputPusherController(gamepad,-1100);
            }
            else {
                setSpinnerVelocity(-1000);
                inputPusherController(gamepad,-1000);
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

    // stop all shooter things
    public void stopSpinners() {
        setSpinnerVelocity(0);
        pusher.setPower(0);
    }

    public void inputPusherAuto(double SPINNER_VELOCITY){
        // Servo control (A = close, default = open)
        if(getSpinnerVelocity() < SPINNER_VELOCITY-20) {
            pusher.setPower(-1);
        }
        else {
            pusher.setPower(0);
        }
    }

    public void LaunchPusherAuto(double SPINNER_VELOCITY){
        if(getSpinnerVelocity() > SPINNER_VELOCITY-20) {
            pusher.setPower(1);
        }
        else {
            pusher.setPower(0);
        }
    }

    /**
     *
     * @param armUp for inputs ( up makes the velocity slower than down).
     * @param launch if true and input is false launch the ball.
     * @param input if true and launch is false intake the ball.
     *              if both are the same spinners stop.
     */
    public void SpinnerAuto(boolean armUp, boolean launch, boolean input, double spinner_velocity){
        if (launch && !input) {            // Launch
            setSpinnerVelocity(spinner_velocity);
            LaunchPusherAuto(spinner_velocity);
        } else if (!launch && input) {      // Intake
            if(!armUp) {
                setSpinnerVelocity(-1100);
                inputPusherAuto(-1100);
            }
            else {
                setSpinnerVelocity(-1000);
                inputPusherAuto(-1000);
            }
        } else {                                // Stop
            stopSpinners();
        }

    }

}





























