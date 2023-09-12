package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class RotatorSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double actualPosition;
    private double targetPosition;

    public RotatorSubsystem(RobotHardware robot) {
        this.robot = robot;
    }


    public void read() {
        actualPosition = robot.rotator.getPosition();
    }

    public void loop() {

    }

    public void write() {
//        if (targetPosition != actualPosition) {
            robot.rotator.setPosition(targetPosition);
//        }
    }


    //setter
    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    public void setPWMRange(PwmControl.PwmRange range) {
        robot.rotator.setPwmRange(range);
    }


    //getters
    public double getPosition() {
        return actualPosition;
    }
}
