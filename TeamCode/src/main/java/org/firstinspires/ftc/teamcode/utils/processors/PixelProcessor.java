package org.firstinspires.ftc.teamcode.utils.processors;

import android.graphics.Canvas;

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

public class PixelProcessor implements VisionProcessor {

    private Mat cameraMatrix;

    private Mat hsv = new Mat();

    private double smaller = 2;

    //contours
    private ArrayList<MatOfPoint> whiteContours = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> greenContours = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> purpleContours = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();
    private ArrayList<ArrayList<MatOfPoint>> contours = new ArrayList<ArrayList<MatOfPoint>>();

    //thresholds
    private Mat whiteThresh = new Mat();
    private Mat greenThresh = new Mat();
    private Mat purpleThresh = new Mat();
    private Mat yellowThresh = new Mat();
    private ArrayList<Mat> threshes = new ArrayList<Mat>();

    //colors
    private Scalar whiteColor = new Scalar(255, 255, 255);
    private Scalar greenColor = new Scalar(44, 223, 16);
    private Scalar purpleColor = new Scalar(152, 0, 140);
    private Scalar yellowColor = new Scalar(255, 255, 3);
    private ArrayList<Scalar> colors = new ArrayList<Scalar>();

    private Scalar whiteLower = new Scalar(0, 0, 0);
    private Scalar whiteUpper = new Scalar(0, 0, 0);

    private Scalar greenLower = new Scalar(0, 0, 0);
    private Scalar greenUpper = new Scalar(0, 0, 0);

    private Scalar purpleLower = new Scalar(129, 18, 108);
    private Scalar purpleUpper = new Scalar(180, 255, 255);

    private Scalar yellowLower = new Scalar(0, 0, 0);
    private Scalar yellowUpper = new Scalar(0, 0, 0);

    private Mat hierarchies = new Mat();

    private PixelCanvasAnnotator canvasAnnotator;

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

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        //white
        Core.inRange(hsv, whiteLower, whiteUpper, threshes.get(0));

        //green
        Core.inRange(hsv, greenLower, greenUpper, threshes.get(1));

        //purple
        Core.inRange(hsv, purpleLower, purpleUpper, threshes.get(2));

        //yellow
        Core.inRange(hsv, yellowLower, yellowUpper, threshes.get(3));


//        for (int i = 0; i < 4; i++) {
//            Imgproc.findContours(threshes.get(2), contours.get(2), new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            for (MatOfPoint contour : contours.get(2)) {
//                Imgproc.drawContours(threshes.get(2), contours.get(2), 0, purpleColor, 2);
//
//                double width = calculateWidth(contour);
//                double height = calculateHeight(contour);
//
//                width -= smaller;
//                height -= smaller;
//
//                Moments moments = Imgproc.moments(contour);
//                double cX = moments.get_m10() / moments.get_m00();
//                double cY = moments.get_m01() / moments.get_m00();
//
//                Imgproc.rectangle(
//                        input,
//                        new Point(cX - (width/2), cY - (height/2)),
//                        new Point(cX + (width/2), cY + (height/2)),
//                        purpleColor,
//                        1
//                );
//            }
//        }

        //purple
        Core.inRange(hsv, purpleLower, purpleUpper, threshes.get(2));
        Imgproc.findContours(threshes.get(2), contours.get(2), hierarchies, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours.get(2)) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);

            width -= 2;
            height -= 2;

//            Moments moments = Imgproc.moments(contour);
//            double cX = moments.get_m10() / moments.get_m00();
//            double cY = moments.get_m01() / moments.get_m00();

//            Imgproc.rectangle(
//                    input,
//                    new Point(cX - (width/2), cY - (height/2)),
//                    new Point(cX + (width/2), cY + (height/2)),
//                    purpleColor,
//                    1
//            );
        }


        hsv.release();
        hierarchies.release();

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


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //canvasAnnotator.noteDrawParams(scaleBmpPxToCanvasPx, scaleCanvasDensity);

        //canvasAnnotator.drawCircle(50,50,50, canvas);
    }

}
