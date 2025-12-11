package org.firstinspires.ftc.teamcode.Math;

public class unitConversion {
    public static double  inTomm(double inch){
        double millimeter = inch * 25.4;

        return millimeter;
    }

    public static double mmTocm(double millimeter){
        double centimeter = millimeter/10;

        return centimeter;
    }

    public static double cmTom(double centimeter){
        double meter = centimeter/100;

        return meter;
    }

    public static double mmTom(double millimeter){
        double meter = millimeter/1000;

        return meter;
    }

    public static double inTom(double inch){
        double millimeter = inTomm(inch);
        double meter = mmTom(millimeter);

        return meter;
    }
}
