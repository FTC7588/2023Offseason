package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.util.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.util.pid.PoofyPIDCoefficients;

import java.util.function.DoubleSupplier;

@Config
public class Constants {

    public static boolean DEBUG_LOOPS = true;

    //control layers
    public static GamepadKeys.Trigger CONTROL_LAYER_2 = GamepadKeys.Trigger.LEFT_TRIGGER;
    public static GamepadKeys.Trigger CONTROL_LAYER_3 = GamepadKeys.Trigger.RIGHT_TRIGGER;

    //drive
    public static PoofyPIDCoefficients DRIVE_X_COEFFS = new PoofyPIDCoefficients(0, 0, 0, 0, 0, 0, 0);
    public static PoofyPIDCoefficients DRIVE_Y_COEFFS = new PoofyPIDCoefficients(0, 0, 0, 0, 0, 0, 0);
    public static PoofyPIDCoefficients DRIVE_THETA_COEFFS = new PoofyPIDCoefficients(1, 0, 0, 0, 0, 0, 0);
    public static double DRIVE_MAX_TURN_SPEED_PID = 8;


    //elevator
    public static PoofyPIDCoefficients ELE_COEFFS = new PoofyPIDCoefficients(0.005, 0.000, 0.0004, 0, 0, 0, 0);

    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 100;


    //arm
    //public static PoofyPIDCoefficients ARM_COEFFS = new PoofyPIDCoefficients(0.01, 0.0001, 0.0001, 0.001, 0, 0.075, 0.125);
    public static PoofyPIDCoefficients ARM_COEFFS = new PoofyPIDCoefficients(0.005, 0, 0, 0, 0, 0, 0.125);

    public static double ARM_MAX_VEL = 100;
    public static double ARM_MAX_ACCEL = 200;

    public static double ARM_ANGLE_BACK = -110;
    public static double ARM_ANGLE_IDLE = 42.5;
    public static double ARM_ANGLE_STACK = 32.5;
    public static double ARM_ANGLE_FRONT = 110;
    public static double ARM_ANGLE_MAX = 130;

    public static double ARM_ENC_BACK_MAX = -720;
    public static double ARM_ENC_BACK_PARALLEL = -520;
    public static double ARM_ENC_FRONT_PARALLEL = 220;
    public static double ARM_ENC_FRONT_MAX = 410;
    public static double ARM_ENC_CENTER = -145;

    public static double ARM_DEADZONE = 10;


    //rotator
    public static PwmControl.PwmRange TUNED_RANGE = new PwmControl.PwmRange(590, 2400);


    //intake
    public static int INTAKE_AVG_SIZE = 75;


    //vision
    public static CameraIntrinsics C920_INTRINSICS = new CameraIntrinsics(504.041, 504.041, 307.462, 234.687);
    public static Pose3d CAMERA_POSE = new Pose3d(
            new Vector3d(-5, 5, 5),
            new Rotation3d(0, 0, 0)
    );
    public static int VISION_AVG = 3;

}
