package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.commandbase.commands.arm.SetArmAngle;
import org.firstinspires.ftc.teamcode.commandbase.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandbase.commands.drive.RobotCentricPID;
import org.firstinspires.ftc.teamcode.commandbase.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandbase.commands.rotator.SetRotatorPosition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.localizers.AprilTagLocalizerSingle;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Basic extends BaseOpMode {

    private RobotCentricPID robotCentricPID;

    private MoveElevatorToPosition eleUp;
    private MoveElevatorToPosition eleDown;

    private SetArmAngle armForward;
    private SetArmAngle armBack;

    private SetRotatorPosition rotForward;
    private SetRotatorPosition rotBack;

    private SetIntakePower intakeIn;
    private SetIntakePower intakeIdle;
    private SetIntakePower intakeOut;

    private AprilTagLocalizerSingle tagLocalizer;

    @Override
    public void initialize() {
        super.initialize();

        //drive commands
        robotCentricPID = new RobotCentricPID(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        //elevator commands
        eleUp = new MoveElevatorToPosition(eleSS, 8);
        eleDown = new MoveElevatorToPosition(eleSS, 4);

        //arm commands
        armBack = new SetArmAngle(armSS, ARM_ANGLE_BACK);
        armForward = new SetArmAngle(armSS, ARM_ANGLE_FRONT);

        //rotator commands
        rotForward = new SetRotatorPosition(rotSS, 1);
        rotBack = new SetRotatorPosition(rotSS, 0);

        //intake commands
        intakeIn = new SetIntakePower(intakeSS, 1);
        intakeIdle = new SetIntakePower(intakeSS, 0);
        intakeOut = new SetIntakePower(intakeSS, -1);


        //ele controls
        gp1(DPAD_UP, 1).whenActive(eleUp);
        gp1(DPAD_DOWN, 1).whenActive(eleDown);

        //arm controls
        gp1(X, 1).whenActive(armBack);
        gp1(B, 1).whenActive(armForward);

        //rotator control
        gp1(Y, 1).toggleWhenActive(rotBack, rotForward);

        //intake controls
        gp1(LEFT_BUMPER, 1).whenActive(intakeOut).whenInactive(intakeIdle);
        gp1(RIGHT_BUMPER, 1).whenActive(intakeIn).whenInactive(intakeIdle);


        //instantiate localizer
        tagLocalizer = new AprilTagLocalizerSingle(robot, CAMERA_POSE, C920_INTRINSICS, VISION_AVG);


        //init commands
        robotCentricPID.schedule();
        rotForward.schedule();

        tal("Ready");
        tau();
    }

    @Override
    public void initLoop() {

    }

    @Override
    public void runInit() {

    }

    @Override
    public void run() {
        super.run();
        robot.read(subsystems);

        robot.loop(subsystems);
        tagLocalizer.update();
        robot.write(subsystems);

        if (tagLocalizer.getTargetTag() != null) {
            tad("x", tagLocalizer.getCamPose().getX());
            tad("y", tagLocalizer.getCamPose().getY());
            tad("z", tagLocalizer.getCamPose().getZ());
            tal();
            tad("filtered roll", tagLocalizer.getCamPose().getRotation().getX());
            tad("filtered pitch", tagLocalizer.getCamPose().getRotation().getY());
            tad("filtered yaw", tagLocalizer.getCamPose().getRotation().getZ());
            tal();
        }
        tau();

        robot.clearBulkCache();
    }
}
