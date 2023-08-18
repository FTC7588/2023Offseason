package org.firstinspires.ftc.teamcode.utils.localizers.butgood;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.utils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.utils.hardware.CameraConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagStreamer {

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    private ArrayList<AprilTagDetection> tags;

    public AprilTagStreamer(CameraConfig config) {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(
                        config.getIntrinsics().getFx(),
                        config.getIntrinsics().getFy(),
                        config.getIntrinsics().getCx(),
                        config.getIntrinsics().getCy()
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(config.getCamera())
                .addProcessor(tagProcessor)
                .setCameraResolution(config.getSize())
                .setStreamFormat(config.getFormat())
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);

        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure((long) config.getExposure(), TimeUnit.MILLISECONDS);

        gain.setGain(config.getGain());

        tags = tagProcessor.getDetections();
    }

    public void update() {
        tags = tagProcessor.getDetections();
    }

    public ArrayList<AprilTagDetection> getTags() {
        return tags;
    }

}
