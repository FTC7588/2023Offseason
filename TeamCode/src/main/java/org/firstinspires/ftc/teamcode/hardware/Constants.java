package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.pid.PoofyPIDCoefficients;

public class Constants {

    public static double ELE_KP = 0.005;
    public static double ELE_KI = 0.0002;
    public static double ELE_KD = 0.0004;
    public static double ELE_KF = 0.175;

    public static PoofyPIDCoefficients ELE_COEFFS= new PoofyPIDCoefficients(0.005, 0.0002, 0.0004, 0, 0, 0, 0.175);

    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 50;

}
