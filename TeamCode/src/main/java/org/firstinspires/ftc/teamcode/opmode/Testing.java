package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.commandbase.commands.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class Testing extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private GamepadEx driver;

    private ElevatorSubsystem elevatorSS;

    private Subsystems subsystems;


    private MoveElevatorToPosition eleUp;
    private MoveElevatorToPosition eleDown;


    protected double loopTime;
    protected RollingAverage loopAvg;


    @Override
    public void initialize() {
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);

        elevatorSS = new ElevatorSubsystem(robot);

        subsystems = new Subsystems(elevatorSS);

        eleUp = new MoveElevatorToPosition(elevatorSS, 8);
        eleDown = new MoveElevatorToPosition(elevatorSS, 4);


        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(eleDown);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(eleUp);


        loopAvg = new RollingAverage(50);

        telemetry.addLine("ready:");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        robot.read(subsystems);

        robot.loop(subsystems);
        robot.write(subsystems);

        telemetry.addData("ele target power", elevatorSS.getEleTargetPower());
        telemetry.addData("ele actual power", elevatorSS.getEleActualPower());
        telemetry.addData("ele target", elevatorSS.getEleGoal());
        telemetry.addData("ele setpoint", elevatorSS.getEleSetPoint());
        telemetry.addData("ele pos", elevatorSS.getElePos());
        telemetry.addLine();
        telemetry.addData("loop time", System.currentTimeMillis() - loopTime);
        telemetry.addData("loop avg", loopAvg.getAverage());
        loopAvg.addNumber((int) (System.currentTimeMillis() - loopTime));
        loopTime = System.currentTimeMillis();
        telemetry.update();

        robot.clearBulkCache();
    }
}
