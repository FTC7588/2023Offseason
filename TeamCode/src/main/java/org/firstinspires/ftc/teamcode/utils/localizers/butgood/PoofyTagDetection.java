package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class PoofyTagDetection {

    public AprilTagDetection tag;
    public Pose3d robotToCamera;

    public Pose2d tagPose2d;
    public Transform2d readingTransform2d;
    public Pose2d cameraPose2d;
    public Pose2d robotPose2d;

    public Pose3d tagPose3d;
    public Transform3d readingTransform3d;
    public Pose3d cameraPose3d;
    public Pose3d robotPose3d;

    protected Transform2d coordinateFix2d = new Transform2d(0, 0, Math.toRadians(90));

    protected Transform3d coordinateFix3d = new Transform3d(
            new Vector3d(0, 0, 0),
            new Rotation3d(0, 0, Math.toRadians(90))
    );

    public PoofyTagDetection(AprilTagDetection tag, Pose3d robotToCamera) {
        this.tag = tag;
        this.robotToCamera = robotToCamera;
//
//        tagPose2d = new Pose2d(
//                tag.metadata.fieldPosition.get(0),
//                tag.metadata.fieldPosition.get(1),
//                MathUtil.quaternionToEuler(tag.metadata.fieldOrientation).yaw + Math.toRadians(90)
//        );
//
//        readingTransform2d = new Transform2d(
//                tag.ftcPose.x,
//                tag.ftcPose.y,
//                Math.toRadians(tag.ftcPose.yaw)
//        );
//
//        cameraPose2d = tagPose2d.transformBy(readingTransform2d.inverse()).transformBy(coordinateFix2d);
//
//        robotPose2d = cameraPose2d.transformBy(robotToCamera.toTransform2d().inverse());

        tagPose3d = new Pose3d(
                new Vector3d(
                        tag.metadata.fieldPosition.get(0),
                        tag.metadata.fieldPosition.get(1),
                        tag.metadata.fieldPosition.get(2)
                ),
                new Rotation3d(
                        tag.metadata.fieldOrientation
                )
        );

        readingTransform3d = new Transform3d(
                new Vector3d(
                        tag.ftcPose.x,
                        tag.ftcPose.y,
                        tag.ftcPose.z
                ),
                new Rotation3d(
                        tag.ftcPose.roll,
                        tag.ftcPose.pitch,
                        tag.ftcPose.yaw
                )
        );


        cameraPose2d = null;

        cameraPose3d = tagPose3d.transformBy(readingTransform3d.inverse()).transformBy(coordinateFix3d);

        robotPose3d = cameraPose3d.transformBy(robotToCamera.toTransform3d().inverse());

        tagPose2d = tagPose3d.toPose2d();
        readingTransform2d = readingTransform3d.toTransform2d();
        cameraPose2d = cameraPose3d.toPose2d();
        robotPose2d = robotPose3d.toPose2d();

    }

    public Pose2d getTagPose2d() {
        return tagPose2d;
    }

    public Transform2d getReadingTransform2d() {
        return readingTransform2d;
    }

    public Pose2d getCameraPose2d() {
        return cameraPose2d;
    }

    public Pose3d getTagPose3d() {
        return tagPose3d;
    }

    public Transform3d getReadingTransform3d() {
        return readingTransform3d;
    }

    public Pose3d getCameraPose3d() {
        return cameraPose3d;
    }
}
