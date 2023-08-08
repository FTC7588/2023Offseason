package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.commandbase.commands.arm.SetArmAngle;
import org.firstinspires.ftc.teamcode.commandbase.commands.drive.FieldCentricPID;
import org.firstinspires.ftc.teamcode.commandbase.commands.drive.FollowTag;
import org.firstinspires.ftc.teamcode.commandbase.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandbase.commands.drive.RobotCentricPID;
import org.firstinspires.ftc.teamcode.commandbase.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandbase.commands.rotator.SetRotatorPosition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Basic extends BaseOpMode {

    private RobotCentricPID robotCentricPID;
    private FieldCentricPID fieldCentricPID;
    private FollowTag followTag;

    private MoveElevatorToPosition eleUp;
    private MoveElevatorToPosition eleDown;

    private SetArmAngle armForward;
    private SetArmAngle armBack;

    private SetRotatorPosition rotForward;
    private SetRotatorPosition rotBack;

    private SetIntakePower intakeIn;
    private SetIntakePower intakeIdle;
    private SetIntakePower intakeOut;


    protected Pose2d followPose = new Pose2d(5, 40, 0);

    @Override
    public void initialize() {
        super.initialize();

        robot.resetIMU();

        //drive commands
        robotCentricPID = new RobotCentricPID(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        fieldCentricPID = new FieldCentricPID(
                driveSS,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()
        );

        followTag = new FollowTag(driveSS, followPose);

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


        //drive controls
        gp1(A, 2).whenActive(followTag);
        gp1(X, 2).whenActive(robotCentricPID);
        gp1(B, 2).whenActive(fieldCentricPID);


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

        //imu controls
        gp1(A, 3).whenActive(robot::resetIMU);


        //init commands
        //robotCentricPID.schedule();
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
        robot.write(subsystems);



        if (driveSS.isDetected()) {
            tad("x", driveSS.getCamPose().getX());
            tad("y", driveSS.getCamPose().getY());
            tad("z", driveSS.getCamPose().getZ());
            tal();
            tad("filtered roll", Math.toDegrees(driveSS.getCamPose().getRotation().getX()));
            tad("filtered pitch", Math.toDegrees(driveSS.getCamPose().getRotation().getY()));
            tad("filtered yaw", Math.toDegrees(driveSS.getCamPose().getRotation().getZ()));
            tal();
        }

        tad("roll", Math.toDegrees(robot.getRoll()));
        tad("pitch", Math.toDegrees(robot.getPitch()));
        tad("heading", Math.toDegrees(robot.getHeading()));

        tad("drive mode", driveSS.getMode());
        tal();
        tau();

        robot.clearBulkCache();
    }
}
