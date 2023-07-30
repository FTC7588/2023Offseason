package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.pid.PoofyProfiledPIDController;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double elePos;
    private double eleGoal;
    private double eleCurrent;
    private double elePower;

    private final PoofyProfiledPIDController controller;

    public ElevatorSubsystem(RobotHardware robot) {
        this.robot = robot;

        MotionProfile.Constraints constraints = new MotionProfile.Constraints(ELE_MAX_VEL, ELE_MAX_ACCEL);

        controller = new PoofyProfiledPIDController(ELE_COEFFS, constraints);
    }

    public void loop() {
        elePower = controller.calculate(elePos);
    }

    public void read() {
        elePos = (robot.eleL.getCurrentPosition() + robot.eleR.getCurrentPosition()) / 2.0;
        eleCurrent = (robot.eleL.getCurrent(CurrentUnit.AMPS) + robot.eleR.getCurrent(CurrentUnit.AMPS)) / 2.0;
    }

    public void write() {
        robot.eleL.setPower(elePower);
        robot.eleR.setPower(elePower);
    }

    public void setTarget(double target) {
        eleGoal = inchesToTicks(target);
        controller.setGoal(eleGoal);
    }


    public double getElePos() {
        return ticksToInches(elePos);
    }

    public double getEleGoal() {
        return ticksToInches(eleGoal);
    }

    public double getEleSetPoint() {
        return ticksToInches(controller.getSetpoint().x);
    }

    public double getEleCurrent() {
        return eleCurrent;
    }

    public double getElePower() {
        return elePower;
    }


    private double inchesToTicks(double inches) {
        return inches * 79.0426072601;
    }

    private double ticksToInches(double ticks) {
        return ticks / 79.0426072601;
    }
}
