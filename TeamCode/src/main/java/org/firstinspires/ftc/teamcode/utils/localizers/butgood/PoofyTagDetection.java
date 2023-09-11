package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class PoofyTagDetection {

    public AprilTagDetection tag;
    public Pose3d robotToCamera;

    public Pose2d tagPose2d;
    public Transform2d readingTransform2d;
    public Pose2d cameraPose2d;
    

    public Pose3d tagPose3d;
    public Transform3d readingTransform3d;
    public Pose3d cameraPose3d;

    public PoofyTagDetection(AprilTagDetection tag, Pose3d robotToCamera) {
        this.tag = tag;
        this.robotToCamera = robotToCamera;

        tagPose2d = new Pose2d(
                tag.metadata.fieldPosition.get(0),
                tag.metadata.fieldPosition.get(1),
                MathUtil.quaternionToEuler(tag.metadata.fieldOrientation).yaw + Math.toRadians(90)
        );

        readingTransform2d = new Transform2d(
                tag.ftcPose.x,
                tag.ftcPose.y,
                Math.toRadians(tag.ftcPose.yaw)
        );

        cameraPose2d
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
