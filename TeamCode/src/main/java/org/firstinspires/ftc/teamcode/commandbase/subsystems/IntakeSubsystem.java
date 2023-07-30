package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double actualPower;
    private double targetPower;

    private double current;
    private final RollingAverage avgCurrent;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
        avgCurrent = new RollingAverage(INTAKE_AVG_SIZE);
    }


    public void read() {
        actualPower = robot.intake.get();
    }

    public void loop() {
        current = robot.expansionHubServoCurrent;
        avgCurrent.addNumber((int) current);
    }

    public void write() {
        if (targetPower != actualPower) {
            robot.intake.set(targetPower);
        }
    }


    //setters and getters
    public void setPower(double power) {
        targetPower = power;
    }

    public double getPower() {
        return actualPower;
    }

    public double getCurrent() {
        return current;
    }

    public RollingAverage getAvgCurrent() {
        return avgCurrent;
    }
}
