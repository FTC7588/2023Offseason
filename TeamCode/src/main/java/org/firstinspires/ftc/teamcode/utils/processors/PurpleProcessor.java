package org.firstinspires.ftc.teamcode.utils.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class PurpleProcessor implements VisionProcessor {

    private Mat hsv = new Mat();

    private ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
    private Mat purpleThresh = new Mat();

    private Scalar purpleColor = new Scalar(152, 0, 140);

    private Scalar purpleLower = new Scalar(129, 18, 108);
    private Scalar purpleUpper = new Scalar(180, 255, 255);

    private Mat hierarchies = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);



        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
