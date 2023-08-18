package org.firstinspires.ftc.teamcode.utils.hardware;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utils.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.utils.geometry.Pose3d;
import org.firstinspires.ftc.vision.VisionPortal;

public class CameraConfig {

    private final CameraName camera;
    private final Pose3d camPose;
    private final CameraIntrinsics intrinsics;
    private final double exposure;
    private final int gain;
    private final Size size;
    private final VisionPortal.StreamFormat format;

    public CameraConfig(
            CameraName camera,
            Pose3d camPose,
            CameraIntrinsics intrinsics,
            double exposure,
            int gain,
            Size size,
            VisionPortal.StreamFormat format
    ) {
        this.camera = camera;
        this.camPose = camPose;
        this.intrinsics = intrinsics;
        this.exposure = exposure;
        this.gain = gain;
        this.size = size;
        this.format = format;
    }

    public CameraName getCamera() {
        return camera;
    }

    public Pose3d getCamPose() {
        return camPose;
    }

    public CameraIntrinsics getIntrinsics() {
        return intrinsics;
    }

    public double getExposure() {
        return exposure;
    }

    public int getGain() {
        return gain;
    }

    public Size getSize() {
        return size;
    }

    public VisionPortal.StreamFormat getFormat() {
        return format;
    }
}
