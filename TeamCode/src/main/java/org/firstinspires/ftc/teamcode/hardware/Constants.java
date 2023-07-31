package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.util.pid.PoofyPIDCoefficients;

@Config
public class Constants {

    //elevator
    public static PoofyPIDCoefficients ELE_COEFFS= new PoofyPIDCoefficients(0.005, 0.000, 0.0004, 0, 0, 0, 0);

    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 100;


    //rotator
    public static PwmControl.PwmRange TUNED_RANGE = new PwmControl.PwmRange(590, 2400);


    //intake
    public static int INTAKE_AVG_SIZE = 75;

}
