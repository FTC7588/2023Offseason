package org.firstinspires.ftc.teamcode.utils.processors;

import android.annotation.SuppressLint;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class PurpleProcessor implements VisionProcessor {

    public Telemetry telemetry;

    private Mat hsv = new Mat();

    private ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
    private Mat purpleThresh = new Mat();

    private Scalar purpleColor = new Scalar(152, 0, 140);

    public Scalar purpleLower = new Scalar(129, 18, 108);
    public Scalar purpleUpper = new Scalar(180, 255, 255);

    private Mat hierarchies = new Mat();

    public PurpleProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @SuppressLint("DefaultLocale")
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        //purple
        Core.inRange(hsv, purpleLower, purpleUpper, purpleThresh);
        Imgproc.findContours(purpleThresh, purpleContours, hierarchies, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : purpleContours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    input,
                    new Point(cX - (width/2), cY - (height/2)),
                    new Point(cX + (width/2), cY + (height/2)),
                    purpleColor,
                    1
            );

            telemetry.addLine(String.format("Contour Point %6.1f, %6.1f", cX, cY));
        }



        telemetry.update();

        purpleContours.clear();

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }
}
