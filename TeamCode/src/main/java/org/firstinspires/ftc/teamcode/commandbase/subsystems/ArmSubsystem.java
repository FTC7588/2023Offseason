package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.pid.PoofyPIDController;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private final PoofyPIDController controller;

    private final InterpLUT angleEncLUT;
    private final InterpLUT encAngleLUT;

    private double armPos;

    private double armPastPower;
    private double armPower;

    public ArmSubsystem(RobotHardware robot) {
        this.robot = robot;

        controller = new PoofyPIDController(ARM_COEFFS);
        controller.enableAngleFeedforward();

        angleEncLUT = new InterpLUT();
        angleEncLUT.add(-ARM_ANGLE_MAX, ARM_ENC_BACK_MAX);
        angleEncLUT.add(-90, ARM_ENC_BACK_PARALLEL);
        angleEncLUT.add(0, ARM_ENC_CENTER);
        angleEncLUT.add(90, ARM_ENC_FRONT_PARALLEL);
        angleEncLUT.add(ARM_ANGLE_MAX, ARM_ENC_FRONT_MAX);
        angleEncLUT.createLUT();

        encAngleLUT = new InterpLUT();
        encAngleLUT.add(ARM_ENC_BACK_MAX, -ARM_ANGLE_MAX);
        encAngleLUT.add(ARM_ENC_BACK_PARALLEL, -90);
        encAngleLUT.add(ARM_ENC_CENTER, 0);
        encAngleLUT.add(ARM_ENC_FRONT_PARALLEL, 90);
        encAngleLUT.add(ARM_ENC_FRONT_MAX, ARM_ANGLE_MAX);
        encAngleLUT.createLUT();
    }

    public void read() {
        armPos = -robot.arm.getCurrentPosition();
    }

    public void loop() {
        controller.setAngle(encAngleLUT.get(armPos));
        armPastPower = armPower;
        armPower = controller.calculate(armPos);
    }

    public void write() {
        robot.arm.setPower(armPower);
    }

    public void setArmAngle(double angleTarget) {
        controller.setTargetPosition(angleEncLUT.get(angleTarget));
    }

    public boolean isFinished() {
        return (controller.getPositionError() < ARM_DEADZONE &&
               -controller.getPositionError() > ARM_DEADZONE);
    }


    //getters


    public double getArmPos() {
        return armPos;
    }

    public double getArmPower() {
        return armPower;
    }
}
