package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorLibrary {
    private Servo pwm_object;

    public double OFF = 0.0;
    public double RED = 0.278;
    public double ORANGE = 0.333;
    public double YELLOW = 0.388;
    public double SAGE = 0.444;
    public double GREEN = 0.500;
    public double AZURE = 0.555;
    public double BLUE = 0.611;
    public double INDIGO = 0.666;
    public double VIOLET = 0.721;
    public double WHITE = 1.0;

    public void ColorServoSetUp(HardwareMap hardwareMap) {
        pwm_object = hardwareMap.get(Servo.class, "LED");
    }

    /**
     *
     * @param color [0,1]
     */
    public void setColor(double color) {
        pwm_object.setPosition(color);
    }

}
