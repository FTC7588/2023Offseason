package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.enums.DriveMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.utils.hardware.CameraConfig;
import org.firstinspires.ftc.teamcode.utils.localizers.AprilTagLocalizer2dSingle;
import org.firstinspires.ftc.teamcode.utils.localizers.butgood.AprilTagLocalizer2d;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import android.util.Size;

public class DrivetrainSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double heading;

    private final MecanumDrive drive;
    private DriveMode mode;

    private Pose2d robotPose;



    private final AprilTagLocalizer2d tagLocalizer;

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

        //tagLocalizer = new AprilTagLocalizer2dSingle(robot, CAMERA_POSE.toPose2d(), C920_INTRINSICS, VISION_AVG);

        tagLocalizer = new AprilTagLocalizer2d(new CameraConfig(
                robot.camera,
                new Pose3d(new Vector3d(0, 0, 0), new Rotation3d(0, 0, 0)),
                C920_INTRINSICS,
                15,
                255,
                new Size(620, 480),
                VisionPortal.StreamFormat.MJPEG
        ));

        robotPose = new Pose2d(0, 0, 0);
    }


    public void read() {

    }

    public void loop() {
        heading = robot.getHeading();
        tagLocalizer.update();
//        if (tagLocalizer.isDetected()) {
            robotPose = tagLocalizer.getPoseEstimate();
//        }
    }

    public void write() {
        drive.write();
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }


//    public Pose2d getTagPose() {
//        return tagLocalizer.getTagPose();
//    }
//
//    public Pose2d getCamPose() {
//        return tagLocalizer.getCamPose();
//    }
//
//    public Transform2d getCamToTag() {
//        return tagLocalizer.getCamToTag();
//    }


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

//    public void followTagMode(Pose2d followPose) {
//        if (isDetected()) {
//            drive.driveFollowTag(
//                    new Pose2d(
//                            getCamPose().getX(),
//                            getCamPose().getY(),
//                            Math.toDegrees(getCamPose().getTheta())
//                    ),
//                    followPose
//            );
//        } else {
//            drive.driveRobotCentric(0, 0, 0);
//        }
//
//        mode = DriveMode.FOLLOW_TAG;
//    }
//
//    public boolean isDetected() {
//        return tagLocalizer.isDetected();
//    }

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
