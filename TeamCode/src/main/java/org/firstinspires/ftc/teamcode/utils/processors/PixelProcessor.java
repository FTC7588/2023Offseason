package org.firstinspires.ftc.teamcode.utils.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;

public class PixelProcessor implements VisionProcessor {

    public Telemetry telemetry;

    private Mat cameraMatrix;

    private Mat hsv = new Mat();

    public double smaller = 2;

    public double targetRatio = 2.2;
    public double ratioTolerance = 0.4;

    //contours
    public ArrayList<MatOfPoint> whiteContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> greenContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> purpleContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();
    public ArrayList<MatOfPoint> blackContours = new ArrayList<>();
    public ArrayList<ArrayList<MatOfPoint>> contours = new ArrayList<ArrayList<MatOfPoint>>();

    //thresholds
    public Mat whiteThresh = new Mat();
    public Mat greenThresh = new Mat();
    public Mat purpleThresh = new Mat();
    public Mat yellowThresh = new Mat();
    public Mat blackThresh = new Mat();
    public ArrayList<Mat> threshes = new ArrayList<Mat>();

    //colors
    public Scalar whiteColor = new Scalar(0, 0, 0);
    public Scalar greenColor = new Scalar(44, 223, 16);
    public Scalar purpleColor = new Scalar(152, 0, 140);
    public Scalar yellowColor = new Scalar(255, 255, 3);
    public Scalar blackColor = new Scalar(0, 255, 255);
    public ArrayList<Scalar> colors = new ArrayList<Scalar>();

    public Scalar whiteLower = new Scalar(0, 0, 177);
    public Scalar whiteUpper = new Scalar(255, 30, 255);

    public Scalar greenLower = new Scalar(47, 44, 53);
    public Scalar greenUpper = new Scalar(72, 255, 255);

    public Scalar purpleLower = new Scalar(129, 18, 108);
    public Scalar purpleUpper = new Scalar(180, 255, 255);

    public Scalar yellowLower = new Scalar(0, 148, 67);
    public Scalar yellowUpper = new Scalar(32, 255, 255);

    public Scalar blackLower = new Scalar(0, 0, 42.5);
    public Scalar blackUpper = new Scalar(190, 142, 97);

    public ArrayList<Scalar> lowers = new ArrayList<>();
    public ArrayList<Scalar> uppers = new ArrayList<>();

    public Mat testThresh = new Mat();
    public ArrayList<MatOfPoint> testContours = new ArrayList<>();

    private Mat hierarchies = new Mat();

    private Mat roi = new Mat();

    private PixelCanvasAnnotator canvasAnnotator;

    public PixelProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        canvasAnnotator = new PixelCanvasAnnotator(cameraMatrix);

        contours.add(whiteContours);
        contours.add(greenContours);
        contours.add(purpleContours);
        contours.add(yellowContours);

        threshes.add(whiteThresh);
        threshes.add(greenThresh);
        threshes.add(purpleThresh);
        threshes.add(yellowThresh);

        colors.add(whiteColor);
        colors.add(greenColor);
        colors.add(purpleColor);
        colors.add(yellowColor);

        lowers.add(whiteLower);
        lowers.add(greenLower);
        lowers.add(purpleLower);
        lowers.add(yellowLower);

        uppers.add(whiteUpper);
        uppers.add(greenUpper);
        uppers.add(purpleUpper);
        uppers.add(yellowUpper);

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, blackLower, blackUpper, blackThresh);
        Imgproc.findContours(blackThresh, blackContours, hierarchies, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : blackContours) {
            double width = calculateWidth(contour) - 2;
            double height = calculateHeight(contour) - 2;

            double size = width*height;
            double ratio = width/height;

            if (within(size, 110, 60) && within(ratio, 2, 1.5)) {
                Moments moments = Imgproc.moments(contour);
                double cX = moments.get_m10() / moments.get_m00();
                double cY = moments.get_m01() / moments.get_m00();

                //determine color
                roi = new Mat(
                        hsv,
                        new Rect(
                                new Point(cX - width*0.8, cY - height*0.8),
                                new Point(cX, cY)
                        )
                );

                for (int i = 0; i < 4; i++) {
                    Core.inRange(roi, lowers.get(i), uppers.get(i), threshes.get(i));

                    Imgproc.findContours(threshes.get(i), contours.get(i), hierarchies, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (MatOfPoint singleContour : contours.get(i)) {
                        double singleWidth = calculateWidth(singleContour);
                        double singleHeight = calculateHeight(singleContour);

                        singleWidth -= 2;
                        singleHeight -= 2;

                        Imgproc.rectangle(
                                input,
                                new Point(cX - (singleWidth*2), cY - (singleHeight*2.5)),
                                new Point(cX + (singleWidth*2), cY + (singleHeight*2.5)),
                                colors.get(i),
                                1
                        );
                    }
                    contours.get(i).clear();
                }
            }
        }

        //clean up
        blackContours.clear();

        telemetry.update();

        return input;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    public boolean within(double num, double tar, double amt) {
        return num + amt > tar && num - amt < tar;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //canvasAnnotator.noteDrawParams(scaleBmpPxToCanvasPx, scaleCanvasDensity);

        //canvasAnnotator.drawCircle(50,50,50, canvas);
    }

}
