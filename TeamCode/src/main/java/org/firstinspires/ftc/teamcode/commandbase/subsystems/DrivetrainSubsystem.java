package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.enums.DriveMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.localizers.AprilTagLocalizerSingle;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double heading;

    private final MecanumDrive drive;
    private DriveMode mode;

    private AprilTagLocalizerSingle tagLocalizer;

    public DrivetrainSubsystem(RobotHardware robot) {
        this.robot = robot;

        drive = new MecanumDrive(
                robot.fL,
                robot.fR,
                robot.rL,
                robot.rR,
                DRIVE_X_COEFFS,
                DRIVE_Y_COEFFS,
                DRIVE_THETA_COEFFS,
                DRIVE_MAX_TURN_SPEED_PID
        );

        tagLocalizer = new AprilTagLocalizerSingle(robot, CAMERA_POSE, C920_INTRINSICS, VISION_AVG);
    }


    public void read() {

    }

    public void loop() {
        heading = robot.getHeading();
        tagLocalizer.update();
    }

    public void write() {
        drive.write();
    }


    public Pose3d getCamPose() {
        return tagLocalizer.getCamPose();
    }


    public void robotCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            drive.driveRobotCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed
            );
        } else {
            drive.driveRobotCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
        mode = DriveMode.ROBOT_CENTRIC;
    }

    public void fieldCentricMode(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean pidTurning) {
        if (!pidTurning) {
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        } else {
            drive.driveFieldCentricPID(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    heading
            );
        }
        mode = DriveMode.FIELD_CENTRIC;
    }

    public void followTagMode(Pose2d followPose) {
        if (isDetected()) {
            drive.driveFollowTag(
                    new Pose2d(
                            getCamPose().getX(),
                            getCamPose().getY(),
                            Math.toDegrees(getCamPose().getRotation().getZ())
                    ),
                    followPose
            );
        } else {
            drive.driveRobotCentric(0, 0, 0);
        }

        mode = DriveMode.FOLLOW_TAG;
    }

    public boolean isDetected() {
        return tagLocalizer.isDetected();
    }

    public DriveMode getMode() {
        return mode;
    }

    public double getHeading() {
        return Math.toDegrees(heading);
    }

    public double getThetaTarget() {
        return drive.getThetaTarget();
    }

    public void setThetaTarget(double thetaTarget) {
        drive.setThetaTarget(thetaTarget);
    }
}
