package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public class PoofyDashboardUtil {

    public static double ROBOT_SIZE = 15;

    public static void drawTags(Canvas canvas, AprilTagLibrary tagLibrary) {
        AprilTagMetadata[] tags = tagLibrary.getAllTags();
        for (AprilTagMetadata tag : tags) {
            Vector2d tagVector = new Vector2d(tag.fieldPosition.get(0), tag.fieldPosition.get(1));
            Vector2d left = new Vector2d(tagVector.getX(), tagVector.getY()).add(new Vector2d(0, tag.tagsize/2).rotateBy(MathUtil.quaternionToEuler(tag.fieldOrientation).yaw));
            Vector2d right = new Vector2d(tagVector.getX(), tagVector.getY()).add(new Vector2d(0, -tag.tagsize/2).rotateBy(MathUtil.quaternionToEuler(tag.fieldOrientation).yaw));
            Vector2d direction = new Vector2d(tagVector.getX(), tagVector.getY()).add(new Vector2d(tag.tagsize/2, 0).rotateBy(MathUtil.quaternionToEuler(tag.fieldOrientation).yaw));

            canvas.setStroke("#ffffff");
            canvas.strokeLine(left.getX(), left.getY(), right.getX(), right.getY());
            canvas.strokeLine(tagVector.getX(), tagVector.getY(), direction.getX(), direction.getY());
        }

    }

    public static void drawRobotPose(Canvas canvas, Pose2d pose) {
        canvas.setStroke("#1079b5");
        canvas.strokeCircle(pose.x, pose.y, ROBOT_SIZE/2);
        Vector2d direction = new Vector2d(pose.x, pose.y).add(new Vector2d(ROBOT_SIZE/2, 0).rotateBy(pose.theta));
        canvas.strokeLine(pose.x, pose.y, direction.getX(), direction.getY());
        resetStroke(canvas);
    }

    public static void resetStroke(Canvas canvas) {
        canvas.setStroke("#000000");
    }
}
