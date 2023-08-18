package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.utils.hardware.CameraConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagLocalizer3d extends AprilTagLocalizer2d {

    public AprilTagLocalizer3d(CameraConfig... configs) {
        super(configs);
    }

    @Override
    public void update() {
        super.update();

        detected = tags.size() > 0;


    }

    protected ArrayList<Pose3d> getCamToTagPoses3d(ArrayList<AprilTagDetection> detections) {
        ArrayList<Pose3d> poses = new ArrayList<>();

        for (AprilTagDetection detection : detections) {

            Pose3d tagPose = new Pose3d(
                    new Vector3d(detection.metadata.fieldPosition),
                    new Rotation3d(detection.metadata.fieldOrientation)
            );

            Transform3d camToTag = new Transform3d(
                    new Vector3d(
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z
                    ),
                    new Rotation3d(
                            Math.toRadians(detection.ftcPose.roll),
                            Math.toRadians(detection.ftcPose.pitch),
                            Math.toRadians(detection.ftcPose.yaw - 90)
                    )
            );

            poses.add(tagPose.transformBy(camToTag.inverse()).transformBy(coordinateFix3d));

        }

        return poses;
    }

}
