package org.firstinspires.ftc.teamcode.utils.localizers;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.utils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.utils.filters.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.geometry.EulerAngles;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Transform3d;
import org.firstinspires.ftc.teamcode.utils.geometry.Vector3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagLocalizer3dSingleIMU implements Localizer {

    private final RobotHardware robot;

    private final Pose3d cameraPose;

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagDetection targetTag;
    private List<AprilTagDetection> tags;

    private Pose3d tagPose;
    private Transform3d camToTag;
    private Pose3d camPose;
    private Transform3d robotToCam;
    private Pose3d robotPose;
    private Pose3d pastRobotPose;

    private Pose2d poseVelocity;

    private Pose3d[] estimates;

    private EulerAngles robotAngles;

    private final MovingAverage rollAverage;
    private final MovingAverage pitchAverage;
    private final MovingAverage yawAverage;

    private final ElapsedTime timer;

    private double lastUpdateTime;

    private double tagTime;

    private boolean detected = false;
    private boolean detectedOnce = false;

    public AprilTagLocalizer3dSingleIMU(
            RobotHardware robot,
            Pose3d cameraPose,
            CameraIntrinsics cameraIntrinsics,
            int visionAverage
    ) {
        this.robot = robot;
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
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        gain.setGain(255);

        rollAverage = new MovingAverage(visionAverage);
        pitchAverage = new MovingAverage(visionAverage);
        yawAverage = new MovingAverage(visionAverage);

        timer = new ElapsedTime();

        pastRobotPose = new Pose3d();

        tags = tagProcessor.getDetections();
    }

    public void update() {
        robotAngles = robot.getRobotAngles();
        tags = tagProcessor.getDetections();

        estimates = new Pose3d[tags.size()];

        //boolean if tags are seen
        detected = tags.size() > 0;
        if (detected) {
            detectedOnce = true;
        }

        //iterate through tags
        if (detected) {
            for (int i = 0; i < tags.size(); i++) {
                AprilTagDetection tag = tags.get(i);

                targetTag = tag;

                if (detected) {

                    tagPose = new Pose3d(
                            new Vector3d(tag.metadata.fieldPosition),
                            new Rotation3d(tag.metadata.fieldOrientation)
                    );

                    camToTag = new Transform3d(
                            new Vector3d(
                                    tag.ftcPose.x,
                                    tag.ftcPose.y,
                                    tag.ftcPose.z
                            ),
                            new Rotation3d(
                                    Math.toRadians(tag.ftcPose.roll),
                                    Math.toRadians(tag.ftcPose.pitch),
                                    -robot.getHeading() - tagPose.getRotation().getZ()
                            )
                    );

                    estimates[i] = tagPose.transformBy(camToTag.inverse());
                }
            }

//            camPose = WeightedAverage.getWeightedAverage(estimates, 2);
            camPose = estimates[0];

//            rollAverage.addNumber(camPose.getRotation().getX());
//            pitchAverage.addNumber(camPose.getRotation().getY());
//            yawAverage.addNumber(camPose.getRotation().getZ());
//
//            camPose = new Pose3d(
//                    camPose.getVector(),
//                    new Rotation3d(
//                            rollAverage.getAverage(),
//                            pitchAverage.getAverage(),
//                            yawAverage.getAverage()
//                    )
//            );

            robotToCam = new Transform3d(
                    cameraPose.getVector(),
                    cameraPose.getRotation()
            );

            robotPose = camPose.transformBy(robotToCam.inverse());

            tagTime = targetTag.frameAcquisitionNanoTime;
        }

        if (detectedOnce) {
            double dt = timer.seconds() - lastUpdateTime;

            poseVelocity = new Pose2d(
                    (robotPose.getX() - pastRobotPose.getX()) / dt,
                    (robotPose.getY() - pastRobotPose.getY()) / dt,
                    robot.getHeadingVelocity()
            );

            lastUpdateTime = timer.seconds();
            pastRobotPose = robotPose;
        }

    }


    //getters
    public AprilTagDetection getTargetTag() {
        return targetTag;
    }

    public Pose3d getTagPose() {
        return tagPose;
    }

    public Transform3d getCamToTag() {
        return camToTag;
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

    public boolean isDetected() {
        return detected;
    }


    //roadrunner methods
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(getRobotPose().toPose2d().x, getRobotPose().toPose2d().y, getRobotPose().toPose2d().theta);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public double getHeading() {
        return robot.getHeading();
    }

    public Double getHeadingVelocity() {
        return robot.getHeadingVelocity();
    }
}
