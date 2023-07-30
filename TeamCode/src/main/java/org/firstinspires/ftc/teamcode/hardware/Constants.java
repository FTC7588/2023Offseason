package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.util.pid.PoofyPIDCoefficients;

public class Constants {

    //elevator
    public static PoofyPIDCoefficients ELE_COEFFS= new PoofyPIDCoefficients(0.005, 0.0002, 0.0004, 0, 0, 0, 0.175);

    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 50;


    //rotator
    public static PwmControl.PwmRange TUNED_RANGE = new PwmControl.PwmRange(590, 2400);


    //intake
    public static int INTAKE_AVG_SIZE = 75;

}
