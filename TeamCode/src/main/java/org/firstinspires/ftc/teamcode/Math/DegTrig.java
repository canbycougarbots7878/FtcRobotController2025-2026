package org.firstinspires.ftc.teamcode.Math;

public class DegTrig {
    public static double sinDeg(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);
        double sin = Math.sin(radAngle);

        return sin;
    }

    public static double cosDeg(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);
        double cos = Math.cos(radAngle);

        return cos;
    }

    public static double tanDeg(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);
        double tan = Math.tan(radAngle);

        return tan;
    }

    public static double secDeg(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);
        double sec = inverseTrig.sec(radAngle);

        return sec;
    }

    public static double csc(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);

        double csc = inverseTrig.csc(radAngle);

        return csc;
    }

    public static double cot(double degreeAngle){
        double radAngle = Math.toRadians(degreeAngle);
        double cot = inverseTrig.cot(radAngle);

        return cot;
    }

}
