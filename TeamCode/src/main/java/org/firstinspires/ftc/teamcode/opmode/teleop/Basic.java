package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.utils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.utils.PoofyDashboardUtil;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    private boolean runOnce = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();


    protected Pose2d followPose = new Pose2d(5, -30, 0);

    @Override
    public void initialize() {
        super.initialize();

        dashboard.setTelemetryTransmissionInterval(100);

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
        //gp1(A, 3).whenActive(robot::resetIMU);


        //init commands
        //robotCentricPID.schedule();
        rotForward.schedule();

        tal("Ready");
        tau();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();
        if (runOnce) {
            //robot.startIMUThread(this);
            runOnce = false;
        }

        robot.read(subsystems);

        robot.loop(subsystems);

        robot.write(subsystems);



//        if (driveSS.isDetected()) {
////            tad("x", driveSS.getCamPose().getX());
////            tad("y", driveSS.getCamPose().getY());
////            tad("z", driveSS.getCamPose().getZ());
////            tal();
//
////            tal(String.format("camPose XYZ %6.1f, %6.1f, %6.1f", driveSS.getCamPose().getX(), driveSS.getCamPose().getY(), driveSS.getCamPose().getZ()));
////            tal(String.format("camPose RPY %6.1f, %6.1f, %6.1f", Math.toDegrees(driveSS.getCamPose().getRotation().getX()),  Math.toDegrees(driveSS.getCamPose().getRotation().getY()),  Math.toDegrees(driveSS.getCamPose().getRotation().getZ())));
////
////            tal();
////
////            tal(String.format("camToTag XYZ %6.1f, %6.1f, %6.1f", driveSS.getCamToTag().getX(), driveSS.getCamToTag().getY(), driveSS.getCamToTag().getZ()));
////            tal(String.format("camToTag RPY %6.1f, %6.1f, %6.1f", Math.toDegrees(driveSS.getCamToTag().getRotation().getX()),  Math.toDegrees(driveSS.getCamToTag().getRotation().getY()),  Math.toDegrees(driveSS.getCamToTag().getRotation().getZ())));
//
//            tal(String.format("tag pose XY T %6.1f, %6.1f, %8.1f", driveSS.getTagPose().getVector().getX(), driveSS.getTagPose().getVector().getY(), Math.toDegrees(driveSS.getTagPose().getTheta())));
//
//            tal(String.format("camToTag XY T %6.1f, %6.1f, %8.1f", driveSS.getCamToTag().getVector().getX(), driveSS.getCamToTag().getVector().getY(), Math.toDegrees(driveSS.getCamToTag().getRotation())));
//
//            tal(String.format("cam pose XY T %6.1f, %6.1f, %8.1f", driveSS.getCamPose().getVector().getX(), driveSS.getCamPose().getVector().getY(), Math.toDegrees(driveSS.getCamPose().getTheta())));
//
//            tal();
//
//            //tad("tag yaw", driveSS.getTagPose().getRotation().getZ());
//
//            tal();
//        }

        tal(String.format("tag pose XY T %6.1f, %6.1f, %8.1f", driveSS.getRobotPose().getVector().getX(), driveSS.getRobotPose().getVector().getY(), Math.toDegrees(driveSS.getRobotPose().getTheta())));

        tad("range", driveSS.getUsedTags().get(0).ftcPose.range);
        tad("bearing", driveSS.getUsedTags().get(0).ftcPose.bearing);


        tad("roll", Math.toDegrees(robot.getRoll()));
        tad("pitch", Math.toDegrees(robot.getPitch()));
        tad("heading", Math.toDegrees(robot.getHeading()));

        //tad("drive mode", driveSS.getMode());
        tal();
        tau();


        TelemetryPacket packet = new TelemetryPacket();

        PoofyDashboardUtil.drawTags(packet.fieldOverlay(), AprilTagCustomDatabase.getSmallLibrary());
        PoofyDashboardUtil.drawRobotPose(packet.fieldOverlay(), driveSS.getRobotPose());

        dashboard.sendTelemetryPacket(packet);


        robot.clearBulkCache();
    }
}
