package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class PixelProcessor implements VisionProcessor {

    private Mat cameraMatrix;
    private Mat grey = new Mat();

    private Mat bnw = new Mat();

    private PixelCanvasAnnotator canvasAnnotator;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        canvasAnnotator = new PixelCanvasAnnotator(cameraMatrix);

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2GRAY);
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvasAnnotator.noteDrawParams(scaleBmpPxToCanvasPx, scaleCanvasDensity);

        //canvasAnnotator.drawCircle(50,50,50, canvas);
    }

}
