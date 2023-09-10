package org.firstinspires.ftc.teamcode.utils.localizers;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.utils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.filters.WeightedAverage;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.utils.localizers.butgood.Localizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagLocalizer2dSingle implements Localizer {

    private final RobotHardware robot;

    private final Pose2d cameraPose;

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagDetection targetTag;
    private List<AprilTagDetection> tags;

    private Transform2d rotationFix;

    private Pose2d tagPose;
    private Transform2d camToTag;
    private Pose2d camPose;
    private Transform2d robotToCam;
    private Pose2d poseEstimate;
    private Pose2d pastPoseEstimate;

    private Pose2d poseVelocity;

    private Pose2d[] estimates;

    private final MovingAverage yawAverage;

    private final ElapsedTime timer;
    private double lastUpdateTime;

    private boolean detected = false;

    public AprilTagLocalizer2dSingle(
            RobotHardware robot,
            Pose2d cameraPose,
            CameraIntrinsics cameraIntrinsics,
            int visionAverage
    ) {
        this.robot = robot;
        this.cameraPose = cameraPose;

        //initialize tag processor
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

        //initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.C930)
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();

        //ensure opmode doesn't start while camera is starting
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            System.out.println("loading camera...");
        }

        //set lighting parameters
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        gain.setGain(255);

        //various initializations
        yawAverage = new MovingAverage(visionAverage);
        timer = new ElapsedTime();
        pastPoseEstimate = new Pose2d(0, 0, 0);
        tags = tagProcessor.getDetections();
        rotationFix = new Transform2d(0, 0, Math.toRadians(90));
    }

    @Override
    public void update() {
        //update detections
        tags = tagProcessor.getDetections();

        //initialize array of tag poses
        estimates = new Pose2d[tags.size()];

        //boolean to check if tags are seen
        detected = tags.size() > 0;

        //iterate through tags
        if (detected) {
            for (int i = 0; i < tags.size(); i++) {
                //get specific tag
                AprilTagDetection tag = tags.get(i);

                targetTag = tag;

                //find pose of tag
                tagPose = new Pose2d(
                        tag.metadata.fieldPosition.get(0),
                        tag.metadata.fieldPosition.get(1),
                        MathUtil.quaternionToEuler(tag.metadata.fieldOrientation).yaw + Math.toRadians(90)
                );

                camToTag = new Transform2d(
                        tag.ftcPose.x,
                        tag.ftcPose.y,
                        Math.toRadians(tag.ftcPose.yaw)
                );

                estimates[i] = tagPose.transformBy(camToTag.inverse()).transformBy(rotationFix);
            }

            camPose = WeightedAverage.getWeightedAverage2d(estimates, 2);
            //camPose = estimates[0];

            robotToCam = new Transform2d(cameraPose.getVector(), cameraPose.getTheta());

            poseEstimate = camPose.transformBy(robotToCam.inverse());

            //calculate delta time
            double dt = timer.seconds() - lastUpdateTime;

            //calculate robot velocity
            poseVelocity = new Pose2d(
                    (poseEstimate.getX() - pastPoseEstimate.getX()) / dt,
                    (poseEstimate.getY() - pastPoseEstimate.getY()) / dt,
                    robot.getHeadingVelocity()
            );

            //post-loop updates
            lastUpdateTime = timer.seconds();
            pastPoseEstimate = poseEstimate;
        }

    }

    public AprilTagDetection getTargetTag() {
        return targetTag;
    }

    public Pose2d getTagPose() {
        return tagPose;
    }

    public Transform2d getCamToTag() {
        return camToTag;
    }

    public Pose2d getCamPose() {
        return camPose;
    }

    public Transform2d getRobotToCam() {
        return robotToCam;
    }

    public Pose2d getRobotPose() {
        return poseEstimate;
    }

    public double getFPS() {
        return visionPortal.getFps();
    }

    public boolean isDetected() {
        return detected;
    }


}
