package org.firstinspires.ftc.teamcode.Math;

public class inverseTrig {
    public static double sec(double Angle){
        double sec = 1/(Math.cos(Angle));

        return sec;
    }

    public static double csc(double Angle){
        double csc = 1/(Math.sin(Angle));

        return csc;
    }

    public static double cot(double Angle){
        double cot = 1/(Math.tan(Angle));

        return cot;
    }
}
