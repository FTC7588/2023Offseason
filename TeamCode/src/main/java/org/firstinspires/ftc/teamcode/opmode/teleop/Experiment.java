package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.commands.drive.TagTrajectory;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.RotatorSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.opmode.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.utils.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.gamepads.GamepadTrigger;
import org.firstinspires.ftc.teamcode.utils.gamepads.TriggerGamepadEx;

@TeleOp
public class Experiment extends CommandOpModeEx {

    protected final RobotHardware robot = RobotHardware.getInstance();

    protected SampleMecanumDrive driveSS;

    protected GamepadEx driver;
    protected TriggerGamepadEx driverEx;

    protected MultipleTelemetry tele;

    protected double loopTime;
    protected MovingAverage loopAvg;


    protected TagTrajectory tagTrajectory;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        driveSS = new SampleMecanumDrive(robot, hardwareMap);

        driver = new GamepadEx(gamepad1);
        driverEx = new TriggerGamepadEx(gamepad1, driver);

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loopAvg = new MovingAverage(50);

        tagTrajectory = new TagTrajectory(driveSS, new Pose2d());

        gp1(A, 1).whenActive(tagTrajectory);

        driveSS.setPoseEstimate(new Pose2d(-3.75, -31.2, Math.toRadians(-23.5)));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        driveSS.update();


        if (Constants.DEBUG_LOOPS) {
            tad("loop avg", loopAvg.getAverage());
            loopAvg.addNumber(System.currentTimeMillis() - loopTime);
            loopTime = System.currentTimeMillis();
        }
        tad("pose", driveSS.getPoseEstimate());
        tau();
    }



    protected void tau() {
        tele.update();
    }
    protected void tal() {
        tele.addLine();
    }
    protected void tal(String caption) {
        tele.addLine(caption);
    }
    protected void tad(String caption, Object value) {
        tele.addData(caption, value);
    }



    protected GamepadButton gp1(GamepadKeys.Button button) {
        return driver.getGamepadButton(button);
    }

    protected GamepadTrigger gp1(GamepadKeys.Trigger trigger) {
        return driverEx.getGamepadTrigger(trigger);
    }

    protected Trigger gp1(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driver.getGamepadButton(button);
        }
    }

    protected Trigger gp1(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driverEx.getGamepadTrigger(trigger);
        }
    }
}
