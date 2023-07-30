package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class Testing extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private ElevatorSubsystem elevatorSS;

    private Subsystems subsystems;

    @Override
    public void initialize() {
        robot.init(hardwareMap);

        elevatorSS = new ElevatorSubsystem(robot);

        subsystems = new Subsystems(elevatorSS);

        telemetry.addLine("ready:");
        telemetry.update();
    }

    @Override
    public void run() {
        robot.read(subsystems);

        robot.loop(subsystems);
        robot.write(subsystems);

        telemetry.addData("Running", 1);
        telemetry.update();
    }
}
