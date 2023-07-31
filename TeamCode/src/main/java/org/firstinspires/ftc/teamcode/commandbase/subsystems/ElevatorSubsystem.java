package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.pid.PoofyProfiledPIDController;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double elePos;
    private double eleGoal;
    private double eleActualPower;
    private double eleTargetPower;

    private final PoofyProfiledPIDController controller;

    public ElevatorSubsystem(RobotHardware robot) {
        this.robot = robot;

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ELE_MAX_VEL, ELE_MAX_ACCEL);

        controller = new PoofyProfiledPIDController(ELE_COEFFS, constraints);
        controller.setTolerance(10);
    }


    public void read() {
        elePos = (robot.eleL.getCurrentPosition() + robot.eleR.getCurrentPosition()) / 2.0;
        eleActualPower = (robot.eleL.getPower() + robot.eleR.getPower()) / 2.0;
//        double a = robot.fL.getVelocity();
//        double b = robot.fR.getVelocity();
//        double c = robot.rL.getVelocity();
//        double d = robot.rR.getVelocity();
//        double e = robot.fL.getCurrent(CurrentUnit.AMPS);
//        double f = robot.fR.getCurrent(CurrentUnit.AMPS);
//        double g = robot.rL.getCurrent(CurrentUnit.AMPS);
//        double h = robot.rR.getCurrent(CurrentUnit.AMPS);
//        double i = robot.fL.getCurrentPosition();
//        double j = robot.fR.getCurrentPosition();
//        double k = robot.rL.getCurrentPosition();
//        double l = robot.rR.getCurrentPosition();
    }

    public void loop() {
        eleTargetPower = controller.calculate(elePos);
    }

    public void write() {
        if (!controller.atGoal()) {
            robot.eleL.setPower(eleTargetPower);
            robot.eleR.setPower(eleTargetPower);
        }
    }

    public void setTarget(double target) {
        eleGoal = inchesToTicks(target);
        controller.setGoal(eleGoal);
    }


    //getters
    public double getElePos() {
        return ticksToInches(elePos);
    }

    public double getEleGoal() {
        return ticksToInches(eleGoal);
    }

    public double getEleSetPoint() {
        return ticksToInches(controller.getSetpoint().position);
    }

    public double getEleTargetPower() {
        return eleTargetPower;
    }

    public double getEleActualPower() {
        return eleActualPower;
    }

    public boolean isAtGoal() {
        return controller.atGoal();
    }


    //conversion
    private double inchesToTicks(double inches) {
        return inches * 79.0426072601;
    }

    private double ticksToInches(double ticks) {
        return ticks / 79.0426072601;
    }
}
