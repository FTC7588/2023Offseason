package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystems;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import java.util.List;

public class RobotHardware {
    public DcMotorEx fL;
    public DcMotorEx fR;
    public DcMotorEx rL;
    public DcMotorEx rR;

    public DcMotorEx eleL;
    public DcMotorEx eleR;

    public ServoImplEx rotator;

    public CRServo intake;

    public BNO055IMU imu;

    private List<LynxModule> hubs;

    public double controlHubServoCurrent;
    public double expansionHubServoCurrent;


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


        //rotator
        rotator = hwMap.get(ServoImplEx.class, "rotator");

        rotator.setDirection(Servo.Direction.REVERSE);

        rotator.setPwmRange(TUNED_RANGE);


        //intake
        intake = new CRServo(hwMap, "intake");

        intake.setInverted(true);

    }

    public void loop(Subsystems subsystems) {
        subsystems.getEle().loop();
    }

    public void read(Subsystems subsystems) {
        //calculateControlServoBusCurrent();
        //calculateExpansionServoBusCurrent();
        subsystems.getEle().read();
    }

    public void write(Subsystems subsystems) {
        subsystems.getEle().write();
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
}
