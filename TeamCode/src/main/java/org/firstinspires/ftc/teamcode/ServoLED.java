package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoLED {
    private Servo pwm_object;

    final double RED = 0.277;
    final double ORANGE = 0.333;
    final double YELLOW = 0.388;
    final double SAGE = 0.444;
    final double GREEN = 0.500;

    public ServoLED(HardwareMap hardwareMap, String name) {
        pwm_object = hardwareMap.get(Servo.class, name);
    }

    public void setColor(double color) {
        pwm_object.setPosition(color);
    }
}


/*

ServoLED led = null;

led = new ServoLED(hardwareMap, "led_servo_thingie");





 */