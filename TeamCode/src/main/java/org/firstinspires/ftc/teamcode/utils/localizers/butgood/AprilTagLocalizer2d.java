package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import org.firstinspires.ftc.teamcode.utils.AprilTagCustomDatabase;
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
import java.util.LinkedHashMap;
import java.util.Map;

public class AprilTagLocalizer2d implements Localizer {

    protected ArrayList<AprilTagStreamer> streamers = new ArrayList<>();

    protected LinkedHashMap<AprilTagDetection, Pose3d> tagsWithCamPoses;

    protected Pose2d robotPose;

    protected Transform2d coordinateFix2d = new Transform2d(0, 0, Math.toRadians(90));
    protected Transform3d coordinateFix3d = new Transform3d(
            new Vector3d(0, 0, 0),
            new Rotation3d(0, 0, Math.toRadians(90))
    );

    protected boolean detected = false;

    public AprilTagLocalizer2d(CameraConfig... configs) {
        for (CameraConfig config : configs) {
            streamers.add(new AprilTagStreamer(config, AprilTagCustomDatabase.getSmallLibrary()));
        }
    }

    @Override
    public void update() {
        consolidateLists();
        if (!tagsWithCamPoses.isEmpty()) {
            robotPose = lowestDecisionMarginStrategy2d(tagsWithCamPoses);
        }
    }


    protected void consolidateLists() {

        tagsWithCamPoses = new LinkedHashMap<>();
        for (int i = 0; i < streamers.size(); i++) {
            streamers.get(i).update();
            for (int j = 0; j < streamers.get(i).getTags().size(); j++) {
                //tags.add(MathUtil.tagTransformCamToRobot(streamers.get(i).getTags().get(j), streamers.get(i).getCamToRobot()));
                tagsWithCamPoses.put(streamers.get(i).getTags().get(j), streamers.get(i).getCamToRobot());
            }
        }
    }

    protected Pose2d getRobotToTagPose2d(AprilTagDetection detection, Pose3d robotToCam) {
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

        Pose2d cameraPose = tagPose.transformBy(camToTag.inverse()).transformBy(coordinateFix2d);

        return cameraPose.transformBy(robotToCam.toTransform2d().inverse());
    }

    protected Pose2d lowestDecisionMarginStrategy2d(LinkedHashMap<AprilTagDetection, Pose3d> detections) {

        double lowestMargin = -1;
        AprilTagDetection lowestMarginTag = null;
        Pose3d lowestMarginCamPose = null;

        for (Map.Entry<AprilTagDetection, Pose3d> entry : detections.entrySet()) {
            if (lowestMargin < entry.getKey().decisionMargin || lowestMargin == -1) {
                lowestMargin = entry.getKey().decisionMargin;
                lowestMarginTag = entry.getKey();
                lowestMarginCamPose = entry.getValue();
            }
        }

        assert lowestMarginTag != null;
        return getRobotToTagPose2d(lowestMarginTag, lowestMarginCamPose);
    }

    public Pose2d getPoseEstimate() {
        return robotPose;
    }
}
