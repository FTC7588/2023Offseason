package org.firstinspires.ftc.teamcode.util.localizers;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.util.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.util.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.util.filters.WeightedAverage;
import org.firstinspires.ftc.teamcode.util.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.util.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.util.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagLocalizerSingle {

    private final Pose3d cameraPose;

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagDetection targetTag;
    private List<AprilTagDetection> tags;

    private Pose3d tagPose;

    private Pose3d camPose;
    private Transform3d robotToCam;
    private Pose3d robotPose;

    private Transform3d camToTarget;

    private Pose3d[] estimates;

    private final MovingAverage rollAverage;
    private final MovingAverage pitchAverage;
    private final MovingAverage yawAverage;

    public AprilTagLocalizerSingle(
            RobotHardware robot,
            Pose3d cameraPose,
            CameraIntrinsics cameraIntrinsics,
            int visionAverage
    ) {
        this.cameraPose = cameraPose;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(
                        cameraIntrinsics.getFx(),
                        cameraIntrinsics.getFy(),
                        cameraIntrinsics.getCx(),
                        cameraIntrinsics.getCy()
                )
                .setTagLibrary(AprilTagCustomDatabase.getSmallLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

        }

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(10, TimeUnit.MILLISECONDS);

        gain.setGain(255);

        rollAverage = new MovingAverage(visionAverage);
        pitchAverage = new MovingAverage(visionAverage);
        yawAverage = new MovingAverage(visionAverage);

        tags = tagProcessor.getDetections();
    }

    public void update() {
        tags = tagProcessor.getDetections();

        //if tags need to be averaged
        if (tags.size() > 1) {
            estimates = new Pose3d[tags.size()];

            for (int i = 0; i < tags.size(); i++) {
                AprilTagDetection tag = tags.get(i);

                if (tag != null) {

                    tagPose = new Pose3d(
                            new Vector3d(tag.metadata.fieldPosition),
                            new Rotation3d(tag.metadata.fieldOrientation)
                    );

                    camToTarget = new Transform3d(
                            new Vector3d(
                                    tag.ftcPose.x,
                                    tag.ftcPose.y,
                                    tag.ftcPose.z
                            ),
                            new Rotation3d(
                                    Math.toRadians(tag.ftcPose.roll),
                                    Math.toRadians(tag.ftcPose.pitch),
                                    Math.toRadians(tag.ftcPose.yaw)
                            )
                    );

                    estimates[i] = tagPose.transformBy(camToTarget.inverse());
                }
            }

            camPose = WeightedAverage.getWeightedAverage(estimates, 2);

            rollAverage.addNumber(camPose.getRotation().getX());
            pitchAverage.addNumber(camPose.getRotation().getY());
            yawAverage.addNumber(camPose.getRotation().getZ());

            camPose = new Pose3d(
                    camPose.getVector(),
                    new Rotation3d(
                            rollAverage.getAverage(),
                            pitchAverage.getAverage(),
                            yawAverage.getAverage()
                    )
            );

            robotToCam = new Transform3d(
                    cameraPose.getVector(),
                    cameraPose.getRotation()
            );

            robotPose = camPose.transformBy(robotToCam.inverse());

        }
        //if only one tag is seen
        else if (tags.size() == 1) {

            targetTag = tags.get(0);

            if (targetTag != null) {

                rollAverage.addNumber(targetTag.ftcPose.roll);
                pitchAverage.addNumber(targetTag.ftcPose.pitch);
                yawAverage.addNumber(targetTag.ftcPose.yaw);

                tagPose = new Pose3d(
                        new Vector3d(targetTag.metadata.fieldPosition),
                        new Rotation3d(targetTag.metadata.fieldOrientation)
                );

                camToTarget = new Transform3d(
                        new Vector3d(
                                targetTag.ftcPose.x,
                                targetTag.ftcPose.y,
                                targetTag.ftcPose.z
                        ),
                        new Rotation3d(
                                Math.toRadians(rollAverage.getAverage()),
                                Math.toRadians(pitchAverage.getAverage()),
                                Math.toRadians(yawAverage.getAverage())
                        )
                );

                camPose = tagPose.transformBy(camToTarget.inverse());

                robotToCam = new Transform3d(
                        cameraPose.getVector(),
                        cameraPose.getRotation()
                );

                robotPose = camPose.transformBy(robotToCam.inverse());
            }
        }
    }


    //getters
    public AprilTagDetection getTargetTag() {
        return targetTag;
    }

    public Pose3d getTagPose() {
        return tagPose;
    }

    public Transform3d getCamToTarget() {
        return camToTarget;
    }

    public Pose3d getCamPose() {
        return camPose;
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    public Pose3d getRobotPose() {
        return robotPose;
    }

    public double getFPS() {
        return visionPortal.getFps();
    }

}
