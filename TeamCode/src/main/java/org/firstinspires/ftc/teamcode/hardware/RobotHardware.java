package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

public class RobotHardware {
    public DcMotorEx fL, fR, rL, rR;

    public DcMotorEx eleL, eleR;

    public DcMotorEx arm;

    public ServoImplEx rotator;

    public CRServo intake;

    public CameraName C930;
    public CameraName C920;

    private List<LynxModule> hubs;

    public double controlHubServoCurrent;
    public double expansionHubServoCurrent;

    public VoltageSensor batteryVoltageSensor;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;

    private double rollOffset;
    private double pitchOffset;
    private double headingOffset;
    private double headingVelocity;

    private EulerAngles angles;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hwMap;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        synchronized (imuLock) {
            imu = hwMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
            imu.initialize(parameters);
//        }


        //drive
        fL = hwMap.get(DcMotorEx.class, "fL");
        fR = hwMap.get(DcMotorEx.class, "fR");
        rL = hwMap.get(DcMotorEx.class, "rL");
        rR = hwMap.get(DcMotorEx.class, "rR");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        rL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //elevator
        eleL = hwMap.get(DcMotorEx.class, "eleL");
        eleR = hwMap.get(DcMotorEx.class, "eleR");

        eleL.setDirection(DcMotorSimple.Direction.REVERSE);
        eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //arm
        arm = hwMap.get(DcMotorEx.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);


        //rotator
        rotator = hwMap.get(ServoImplEx.class, "rotator");

        rotator.setDirection(Servo.Direction.REVERSE);

        rotator.setPwmRange(TUNED_RANGE);


        //intake
        intake = new CRServo(hwMap, "intake");

        intake.setInverted(true);


        angles = new EulerAngles(
                imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        C930 = hwMap.get(WebcamName.class, "Webcam 1");
        C920 = hwMap.get(WebcamName.class, "Webcam 2");

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public void read(Subsystems subsystems) {
        //calculateControlServoBusCurrent();
        //calculateExpansionServoBusCurrent();

        //headingVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        angles.roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
        angles.pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
        angles.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        subsystems.getDrive().read();
        subsystems.getEle().read();
        subsystems.getArm().read();
        subsystems.getRot().read();
        subsystems.getIntake().read();
    }

    public void loop(Subsystems subsystems) {
        subsystems.getDrive().loop();
        subsystems.getEle().loop();
        subsystems.getArm().loop();
        subsystems.getRot().loop();
        subsystems.getIntake().loop();
    }

    public void write(Subsystems subsystems) {
        subsystems.getDrive().write();
        subsystems.getEle().write();
        subsystems.getArm().write();
        subsystems.getRot().write();
        subsystems.getIntake().write();
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void calculateControlServoBusCurrent() {
        LynxGetADCCommand.Channel servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        LynxGetADCCommand servoCommand = new LynxGetADCCommand(hubs.get(0), servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse servoResponse = servoCommand.sendReceive();
            controlHubServoCurrent = servoResponse.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException ignored) {
        }
    }

    public void calculateExpansionServoBusCurrent() {
        LynxGetADCCommand.Channel servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        LynxGetADCCommand servoCommand = new LynxGetADCCommand(hubs.get(1), servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse servoResponse = servoCommand.sendReceive();
            expansionHubServoCurrent = servoResponse.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException ignored) {
        }
    }

//    public void startIMUThread(LinearOpMode opMode) {
//        imuThread = new Thread(() -> {
//            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
//                synchronized (imuLock) {
//                    angles.roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
//                    angles.pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
//                    angles.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                }
//            }
//        });
//        imuThread.start();
//    }

    public HardwareMap getHwMap() {
        return hwMap;
    }

    public double getRoll() {
        return angles.roll - rollOffset;
    }

    public double getPitch() {
        return angles.pitch - pitchOffset;
    }

    public double getHeading() {
        return angles.yaw - headingOffset;
    }

    public double getHeadingVelocity() {
        return headingVelocity;
    }

    public double getIMUTime() {
        return imu.getRobotYawPitchRollAngles().getAcquisitionTime();
    }

    public void resetIMU() {
        rollOffset = angles.roll;
        pitchOffset = angles.pitch;
        headingOffset = angles.yaw;

    }

    public EulerAngles getRobotAngles() {
        return new EulerAngles(getRoll(), getPitch(), getHeading());
    }
}
