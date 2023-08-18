package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.teamcode.utils.hardware.CameraConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagLocalizer2d implements Localizer {

    protected ArrayList<AprilTagDetection> tags;
    protected ArrayList<AprilTagStreamer> streamers;
    protected ArrayList<Pose3d> camPoses;
    protected ArrayList<Pose2d> camPoseEstimates;

    protected Pose2d camPose;

    protected Transform2d coordinateFix2d = new Transform2d(0, 0, Math.toRadians(90));
    protected Transform3d coordinateFix3d = new Transform3d(
            new Vector3d(0, 0, 0),
            new Rotation3d(0, 0, Math.toRadians(90))
    );

    protected boolean detected = false;

    public AprilTagLocalizer2d(CameraConfig... configs) {
        for (CameraConfig config : configs) {
            streamers.add(new AprilTagStreamer(config));
            camPoses.add(config.getCamPose());
        }
    }

    @Override
    public void update() {
        consolidateLists();
        camPose = lowestDecisionMarginStrategy2d(tags);
    }

    public Pose2d getPoseEstimate() {
        return camPose;
    }


    protected void consolidateLists() {
        tags = new ArrayList<>();
        for (AprilTagStreamer streamer : streamers) {
            streamer.update();
            tags.addAll(streamer.getTags());
        }
    }

    protected Pose2d getCamToTagPose2d(AprilTagDetection detection) {
        Pose2d tagPose = new Pose2d(
                detection.metadata.fieldPosition.get(0),
                detection.metadata.fieldPosition.get(1),
                MathUtil.quaternionToEuler(detection.metadata.fieldOrientation).yaw + Math.toRadians(90)
        );

        Transform2d camToTag = new Transform2d(
                detection.ftcPose.x,
                detection.ftcPose.y,
                Math.toRadians(detection.ftcPose.yaw)
        );

        return tagPose.transformBy(camToTag.inverse()).transformBy(coordinateFix2d);
    }


//    protected ArrayList<Pose2d> getCamToTagPoses2d(ArrayList<AprilTagDetection> detections) {
//        ArrayList<Pose2d> poses = new ArrayList<>();
//
//        for (AprilTagDetection detection : detections) {
//            poses.add(getCamToTagPose2d(detection));
//        }
//
//        return poses;
//    }

    protected Pose2d lowestDecisionMarginStrategy2d(ArrayList<AprilTagDetection> detections) {
        double lowestMargin = 10;

        AprilTagDetection lowestMarginTag = null;

        for (AprilTagDetection detection : detections) {
            if (detection.decisionMargin < lowestMargin) {
                lowestMargin = detection.decisionMargin;
                lowestMarginTag = detection;
            }
        }

        return lowestMarginTag != null ? getCamToTagPose2d(lowestMarginTag) : new Pose2d(0, 0, 0);
    }
}
