package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.pid.PoofyProfiledPIDController;
import org.firstinspires.ftc.teamcode.util.pid.PoofyPIDController;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double elePos;
    private double eleGoal;
    private double eleCurrent;
    private double eleActualPower;
    private double eleTargetPower;

    private final PoofyProfiledPIDController profiledController;

    public ElevatorSubsystem(RobotHardware robot) {
        this.robot = robot;

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ELE_MAX_VEL, ELE_MAX_ACCEL);

        profiledController = new PoofyProfiledPIDController(ELE_COEFFS, constraints);
        profiledController.setTolerance(10);
    }


    public void read() {
        elePos = (robot.eleL.getCurrentPosition() + robot.eleR.getCurrentPosition()) / 2.0;
        eleCurrent = (robot.eleL.getCurrent(CurrentUnit.AMPS) + robot.eleR.getCurrent(CurrentUnit.AMPS)) / 2.0;
        eleActualPower = (robot.eleL.getPower() + robot.eleR.getPower()) / 2.0;
    }

    public void loop() {
        eleTargetPower = profiledController.calculate(elePos);
    }

    public void write() {
        if (!profiledController.atGoal()) {
            robot.eleL.setPower(eleTargetPower);
            robot.eleR.setPower(eleTargetPower);
        }
    }

    public void setTarget(double target) {
        eleGoal = inchesToTicks(target);
        //controller.setTargetPosition(eleGoal);
        profiledController.setGoal(eleGoal);
    }


    //getters
    public double getElePos() {
        return ticksToInches(elePos);
    }

    public double getEleGoal() {
        return ticksToInches(eleGoal);
    }

    public double getEleSetPoint() {
        return ticksToInches(profiledController.getSetpoint().position);
    }

    public double getEleCurrent() {
        return eleCurrent;
    }

    public double getEleTargetPower() {
        return eleTargetPower;
    }

    public double getEleActualPower() {
        return eleActualPower;
    }

    public boolean isAtGoal() {
        return profiledController.atGoal();
    }


    //conversion
    private double inchesToTicks(double inches) {
        return inches * 79.0426072601;
    }

    private double ticksToInches(double ticks) {
        return ticks / 79.0426072601;
    }
}
